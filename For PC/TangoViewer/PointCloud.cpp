#include "PointCloud.h"

namespace kukdh1
{
	glm::vec3 operator+(pcl::PointXYZRGB &lhs, const pcl::PointXYZRGB &rhs)
	{
		return glm::vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
	}

	template <typename PointT>
	void TransformPoint(PointT &input, PointT &translate, Eigen::Matrix3f &rotate)
	{
		Eigen::Vector3f eigenPoint;

		eigenPoint(0) = input.x;
		eigenPoint(1) = input.y;
		eigenPoint(2) = input.z;

		eigenPoint = rotate * eigenPoint;

		input.x = eigenPoint(0) + translate.x;
		input.y = eigenPoint(1) + translate.y;
		input.z = eigenPoint(2) + translate.z;
	}

	template<typename PointT>
	void TransformPoint(PointT & input, glm::mat4 & transform)
	{
		glm::vec4 v4Temp;

		v4Temp.x = input.x;
		v4Temp.y = input.y;
		v4Temp.z = input.z;
		v4Temp.w = 1;

		v4Temp = transform * v4Temp;

		input.x = v4Temp.x;
		input.y = v4Temp.y;
		input.z = v4Temp.z;
	}

	void TransformPoint(Eigen::Vector3f &input, glm::mat4 &transform)
	{
		glm::vec4 v4Temp;

		v4Temp.x = input(0);
		v4Temp.y = input(1);
		v4Temp.z = input(2);
		v4Temp.w = 1;

		v4Temp = transform * v4Temp;

		input(0) = v4Temp.x;
		input(1) = v4Temp.y;
		input(2) = v4Temp.z;
	}

	template <typename PointT>
	void TransformBoundingBox(BoundingBox<PointT> &input, pcl::PointXYZRGB &translate, Eigen::Matrix3f &rotate)
	{
		input.Point[1] = input.Point[0];
		input.Point[1].y = input.Point[6].y;
		input.Point[2] = input.Point[0];
		input.Point[2].x = input.Point[6].x;
		input.Point[2].y = input.Point[6].y;
		input.Point[3] = input.Point[0];
		input.Point[3].x = input.Point[6].x;

		input.Point[4] = input.Point[6];
		input.Point[4].x = input.Point[0].x;
		input.Point[4].y = input.Point[0].y;
		input.Point[5] = input.Point[6];
		input.Point[5].x = input.Point[0].x;
		input.Point[7] = input.Point[6];
		input.Point[7].y = input.Point[0].y;

		for (int i = 0; i < 8; i++)
			TransformPoint(input.Point[i], translate, rotate);
	}

	void TransformPlane(pcl::ModelCoefficients::Ptr pclPlane, glm::mat4 & transform)
	{
		//http://stackoverflow.com/questions/7685495/transforming-a-3d-plane-by-4x4-matrix
		//Pseudo code in above page is wrong

		glm::vec3 v3O;		//Point on the Plane = (-d) / (a^2 + b^2 + c^2) * (a, b, c)
		glm::vec3 v3N;		//Normal Vector of Plane
		glm::vec4 v4O;
		glm::vec4 v4N;

		v3N.x = pclPlane->values[0];
		v3N.y = pclPlane->values[1];
		v3N.z = pclPlane->values[2];
		v3N = glm::normalize(v3N);
		v3O = v3N * -pclPlane->values[3];

		float temp = glm::dot(v3O, v3N) + pclPlane->values[3];

		v4O = glm::vec4(v3O, 1);
		v4N = glm::vec4(v3N, 0);
		v4O = transform * v4O;
		v4N = glm::transpose(glm::inverse(transform)) * v4N;
		v3O = glm::vec3(v4O);
		v3N = glm::vec3(v4N);

		pclPlane->values[0] = v4N.x;
		pclPlane->values[1] = v4N.y;
		pclPlane->values[2] = v4N.z;
		pclPlane->values[3] = -glm::dot(v3O, v3N);
	}

	void DrawCube(BoundingBox<pcl::PointXYZRGB> &pclRegion, COLORREF color)
	{
		int index[24] = { 0, 1, 0, 3, 0, 4, 1, 2, 1, 5, 2, 6, 2, 3, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4 };

		glBegin(GL_LINES);

		glColor3ub(GetRValue(color), GetGValue(color), GetBValue(color));

		for (int i = 0; i < 12; i++)
		{
			glVertex3f(pclRegion.Point[index[i * 2]].x, pclRegion.Point[index[i * 2]].y, pclRegion.Point[index[i * 2]].z);
			glVertex3f(pclRegion.Point[index[i * 2 + 1]].x, pclRegion.Point[index[i * 2 + 1]].y, pclRegion.Point[index[i * 2 + 1]].z);
		}

		glEnd();
	}

	glm::vec3 CalculateCenterOfBoundingBox(BoundingBox<pcl::PointXYZRGB>& pclRegion)
	{
		return (pclRegion.Point[0] + pclRegion.Point[6]) / 2.f;
	}

	pcl::PointXYZRGB & PointCloud::GetPointCloudOrigin()
	{
		return pclOrigin;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & PointCloud::GetPointCloudData()
	{
		return pclPointCloudUnordered;
	}

	glm::mat4 & PointCloud::GetAdjustMatrix()
	{
		return m4Adjust;
	}
	
	PointCloud::PointCloud()
	{
		pclPointCloudUnordered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		fTimestamp = 0.0;

		bFiltered = FALSE;
		bOBBCalculated = FALSE;
		bPlaneCalculated = FALSE;
	}

	PointCloud::~PointCloud()
	{
		pclPointCloudUnordered->clear();
	}

	void PointCloud::DrawOnGLWindow(BOOL bDrawOrigin, BOOL bDrawBoundingBox, BOOL bDrawPlanes)
	{
		size_t stSize;

		glPushMatrix();

		glMultMatrixf(glm::value_ptr(m4Adjust));

		if (bDrawOrigin)
		{
			glBegin(GL_LINES);

			glColor3ub(255, 0, 0);
			glVertex3f(pclOrigin.x - 0.1, pclOrigin.y, pclOrigin.z);
			glVertex3f(pclOrigin.x + 0.1, pclOrigin.y, pclOrigin.z);

			glColor3ub(0, 255, 0);
			glVertex3f(pclOrigin.x, pclOrigin.y - 0.1, pclOrigin.z);
			glVertex3f(pclOrigin.x, pclOrigin.y + 0.1, pclOrigin.z);

			glColor3ub(0, 0, 255);
			glVertex3f(pclOrigin.x, pclOrigin.y, pclOrigin.z - 0.1);
			glVertex3f(pclOrigin.x, pclOrigin.y, pclOrigin.z + 0.1);

			glEnd();
		}

		glPointSize(3.0f);
		glBegin(GL_POINTS);

		stSize = pclPointCloudUnordered->size();

		for (unsigned int i = 0; i < stSize; i++)
		{
			pcl::PointXYZRGB &pclPointXYZRGB = pclPointCloudUnordered->at(i);

			glColor3ub(pclPointXYZRGB.b, pclPointXYZRGB.g, pclPointXYZRGB.r);

			glVertex3f(pclPointXYZRGB.x, pclPointXYZRGB.y, pclPointXYZRGB.z);
		}

		glEnd();

		if (bDrawBoundingBox)
		{
			DrawCube(pclBoundingBox, RGB(100, 100, 100));
		}

		if (bDrawPlanes)
		{
			size_t stPlane;

			stPlane = pclPlaneInfo.size();
			for (size_t i = 0; i < stPlane; i++)
				DrawCube(pclPlaneInfo.at(i).pclBoundingBox, RGB(150, 0, 0));
		}

		glPopMatrix();
	}

	void * PointCloud::operator new(size_t i)
	{
		return _aligned_malloc(i, 16);
	}

	void PointCloud::operator delete(void *ptr)
	{
		_aligned_free(ptr);
	}

	void PointCloud::AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix)
	{
		m4Adjust = m4AdjustMatrix * m4Adjust;
	}

	void PointCloud::ResetTransformMatrix()
	{
		m4Adjust = glm::mat4(1.0f);
	}

	void PointCloud::SaveTransformMatrix()
	{
		size_t stSize;

		stSize = pclPointCloudUnordered->size();

		concurrency::parallel_for(size_t(0), stSize, [&](size_t i)
		{
			TransformPoint(pclPointCloudUnordered->at(i), m4Adjust);
		});

		stSize = pclPlaneInfo.size();

		concurrency::parallel_for(size_t(0), stSize, [&](size_t i)
		{
			TransformPoint(pclPlaneInfo.at(i).pclCentroid, m4Adjust);
			TransformPlane(pclPlaneInfo.at(i).pclModelCoeff, m4Adjust);

			for (int j = 0; j < 8; j++)
				TransformPoint(pclPlaneInfo.at(i).pclBoundingBox.Point[j], m4Adjust);

			size_t stPoints = pclPlaneInfo.at(i).pclPoints->size();
			for (size_t j = 0; j < stPoints; j++)
				TransformPoint(pclPlaneInfo.at(i).pclPoints->at(j), m4Adjust);
		});

		for (int i = 0; i < 8; i++)
			TransformPoint(pclBoundingBox.Point[i], m4Adjust);

		TransformPoint(pclOrigin, m4Adjust);

		ResetTransformMatrix();
	}

	void PointCloud::CalculateBoundingBox()
	{
		if (!bOBBCalculated)
		{
			//Calculate Object Bounding Box for entire point cloud
			Eigen::Matrix3f eigenRotationalMatrix;
			pcl::PointXYZRGB pclPositionVector;
			pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> pclMIE;

			pclMIE.setInputCloud(pclPointCloudUnordered);
			pclMIE.compute();
			pclMIE.getOBB(pclBoundingBox.Point[0], pclBoundingBox.Point[6], pclPositionVector, eigenRotationalMatrix);
			TransformBoundingBox(pclBoundingBox, pclPositionVector, eigenRotationalMatrix);

			//Calculate OBB for each plane
			size_t stSize;

			stSize = pclPlaneInfo.size();

			for (size_t i = 0; i < stSize; i++)
			{
				BoundingBox<pcl::PointXYZRGB> & pclOBB = pclPlaneInfo.at(i).pclBoundingBox;
				Eigen::Vector3f & eigenCenter = pclPlaneInfo.at(i).pclCentroid;

				pclMIE.setInputCloud(pclPlaneInfo.at(i).pclPoints);
				pclMIE.compute();
				pclMIE.getMassCenter(eigenCenter);
				pclMIE.getOBB(pclOBB.Point[0], pclOBB.Point[6], pclPositionVector, eigenRotationalMatrix);
				TransformBoundingBox(pclOBB, pclPositionVector, eigenRotationalMatrix);
			}

			bOBBCalculated = TRUE;
		}
	}

	float PointCloud::GetTimestamp()
	{
		return fTimestamp;
	}

	void PointCloud::ApplyStaticalOutlierRemoveFilter()
	{
		if (!bFiltered)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pclSOR;

			pclSOR.setInputCloud(pclPointCloudUnordered);
			pclSOR.setMeanK(SOR_MEAN_K);
			pclSOR.setStddevMulThresh(SOR_STDDEV_MUL_THRES);
			pclSOR.filter(*pclOutput);

			pclPointCloudUnordered = pclOutput;

			bFiltered = TRUE;
		}
	}

	size_t PointCloud::ExecutePlaneSegmentation()
	{
		//http://www.pointclouds.org/documentation/tutorials/extract_indices.php

		if (!bPlaneCalculated)
		{
			pcl::PointIndices::Ptr pclInliers(new pcl::PointIndices());
			pcl::SACSegmentation<pcl::PointXYZRGB> pclSACSeg;
			pcl::ExtractIndices<pcl::PointXYZRGB> pclExtract;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*pclPointCloudUnordered));
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudF(new pcl::PointCloud<pcl::PointXYZRGB>);

			size_t stPointCount;

			pclSACSeg.setOptimizeCoefficients(TRUE);
			pclSACSeg.setModelType(pcl::SACMODEL_PLANE);
			pclSACSeg.setMethodType(pcl::SAC_RANSAC);
			pclSACSeg.setMaxIterations(SAC_SEG_MAX_ITERATION);
			pclSACSeg.setDistanceThreshold(SAC_SEG_DISTANCE_THRESHOLD);
			pclSACSeg.setEpsAngle(SAC_SEG_ANGLE_THRESHOLD);

			stPointCount = pclCloud->size();

			while (pclCloud->size() > SAC_SEG_RATIO * stPointCount)
			{
				size_t stIndices;
				PlaneInfo pclPlane;

				pclPlane.pclModelCoeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
				pclPlane.pclPoints = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

				pclSACSeg.setInputCloud(pclCloud);
				pclSACSeg.segment(*pclInliers, *pclPlane.pclModelCoeff);

				stIndices = pclInliers->indices.size();

				if (stIndices == 0)
					break;

				pclExtract.setInputCloud(pclCloud);
				pclExtract.setIndices(pclInliers);
				pclExtract.setNegative(FALSE);
				pclExtract.filter(*pclPlane.pclPoints);
				pclExtract.setNegative(TRUE);
				pclExtract.filter(*pclCloudF);

				pclCloud.swap(pclCloudF);

				//Save
				pclPlaneInfo.push_back(pclPlane);
			}

			bPlaneCalculated = TRUE;

			return pclPlaneInfo.size();
		}

		return 0;
	}

	BOOL PointCloud::FromFile(WCHAR * pszFilePath)
	{
		FileIO fiFile;

		bFiltered = FALSE;
		bOBBCalculated = FALSE;
		bPlaneCalculated = FALSE;

		return fiFile.ReadFromFile(pszFilePath, pclPointCloudUnordered, &fTimestamp, &pclOrigin);
	}

	BOOL PointCloud::ToFile(WCHAR * pszFilePath)
	{
		FileIO fiFile;

		SaveTransformMatrix();

		return fiFile.WriteToFile(pszFilePath, FLAG_NO_MUL | FLAG_COLOR_DATA, pclPointCloudUnordered);	//No Mul | No ij
	}

	std::vector<PlaneInfo> & PointCloud::GetPlaneInfo()
	{
		return pclPlaneInfo;
	}

	BoundingBox<pcl::PointXYZRGB> & PointCloud::GetPointCloudOBB()
	{
		return pclBoundingBox;
	}

	BOOL PointCloud::GetFilterStatus()
	{
		return bFiltered;
	}

	BOOL PointCloud::GetOBBStatus()
	{
		return bOBBCalculated;
	}

	BOOL PointCloud::GetPlaneSegmentationStatus()
	{
		return bPlaneCalculated;
	}
}