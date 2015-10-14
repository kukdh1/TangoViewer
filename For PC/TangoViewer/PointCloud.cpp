#include "PointCloud.h"

namespace kukdh1
{
	PointCloud::PointCloud()
	{
		pclPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		pclPointCloudUnordered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	PointCloud::~PointCloud()
	{
		size_t stSize;

		pclPointCloud->clear();
		pclPointCloudUnordered->clear();

		stSize = pclPlaneCoeffs.size();
		for (size_t i = 0; i < stSize; i++)
			pclPlaneCoeffs.at(i).reset();

		stSize = pclPlanePoints.size();
		for (size_t i = 0; i < stSize; i++)
			pclPlanePoints.at(i)->clear();
	}

	void PointCloud::DrawOnGLWindow(BOOL bDrawPlanes)
	{
		size_t stSize;

		glPushMatrix();

		glMultMatrixf(glm::value_ptr(m4Adjust));

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

		if (bDrawPlanes)
		{
			size_t stPlane;
			size_t stPoint;

			glBegin(GL_POINTS);

			stPlane = pclPlanePoints.size();

			for (size_t i = 0; i < stPlane; i++)
			{
				stPoint = pclPlanePoints.at(i)->size();

				glColor3ub(255 * (i % 3 == 0 ? 1 : 0), 255 * (i % 3 == 1 ? 1 : 0), 255 * (i % 3 == 2 ? 1 : 0));

				for (size_t j = 0; j < stPoint; j++)
				{
					pcl::PointXYZRGB &pclPointXYZRGB = pclPlanePoints.at(i)->at(j);

					glVertex3f(pclPointXYZRGB.x + (i + 1) * 1, pclPointXYZRGB.y, pclPointXYZRGB.z);
				}
			}

			glEnd();
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
		m4Adjust *= m4AdjustMatrix;
	}

	void PointCloud::ApplyStaticalOutlierRemoveFilter()
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pclSOR;

		pclSOR.setInputCloud(pclPointCloudUnordered);
		pclSOR.setMeanK(SOR_MEAN_K);
		pclSOR.setStddevMulThresh(SOR_STDDEV_MUL_THRES);
		pclSOR.filter(*pclOutput);

		pclPointCloudUnordered = pclOutput;
	}

	size_t PointCloud::ExecutePlaneSegmentation()
	{
		/*
		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> pclOMPS;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> pclIINE;
		pcl::PointCloud<pcl::Normal>::Ptr pclNormalCloud(new pcl::PointCloud<pcl::Normal>);

		//Estimate Normals
		pclIINE.setNormalEstimationMethod(pclIINE.IINE_METHOD);
		pclIINE.setMaxDepthChangeFactor(IINE_MAX_DEPTH_FACTOR);
		pclIINE.setNormalSmoothingSize(IINE_SMOOTH_FACTOR);
		pclIINE.setInputCloud(pclPointCloud);
		pclIINE.compute(*pclNormalCloud);

		//Segment Planes
		pclOMPS.setMinInliers(OMPS_MINIMAL_INLIERS_PLANE);
		pclOMPS.setAngularThreshold(OMPS_ANGULAR_THRESHOLD);
		pclOMPS.setDistanceThreshold(OMPS_DISTANCE_THRESHOLD);
		pclOMPS.setMaximumCurvature(OMPS_MAXIMUM_CULVATURE);
		pclOMPS.setInputNormals(pclNormalCloud);
		pclOMPS.setInputCloud(pclPointCloud);
		pclOMPS.segment(pclRegion);
		
		return pclRegion.size();
		*/

		//http://www.pointclouds.org/documentation/tutorials/extract_indices.php

		pcl::ModelCoefficients::Ptr pclModelCoeff(new pcl::ModelCoefficients());
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

		stPointCount = pclCloud->size();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPlaneIndices;

		while (pclCloud->size() > SAC_SEG_RATIO * stPointCount)
		{
			size_t stIndices;

			pclPlaneIndices = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

			pclSACSeg.setInputCloud(pclCloud);
			pclSACSeg.segment(*pclInliers, *pclModelCoeff);

			stIndices = pclInliers->indices.size();

			if (stIndices == 0)
				break;

			pclExtract.setInputCloud(pclCloud);
			pclExtract.setIndices(pclInliers);
			pclExtract.setNegative(FALSE);
			pclExtract.filter(*pclPlaneIndices);
			pclExtract.setNegative(TRUE);
			pclExtract.filter(*pclCloudF);

			pclCloud.swap(pclCloudF);

			//Save
			pclPlaneCoeffs.push_back(pclModelCoeff);
			pclPlanePoints.push_back(pclPlaneIndices);

			pclModelCoeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
		}

		return pclPlaneCoeffs.size();
	}

	BOOL PointCloud::FromFile(WCHAR * pszFilePath)
	{
		HANDLE hFile;
		DWORD dwRead;
		Pointij *ppPointij;
		glm::mat4 m4Temp;
		unsigned int uiPointCount;

		hFile = CreateFile(pszFilePath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

		if (hFile != INVALID_HANDLE_VALUE)
		{
			float fMatrix[16];

			ReadFile(hFile, &uiPointCount, 4, &dwRead, NULL);

			if (uiPointCount)
			{
				ReadFile(hFile, fMatrix, 16 * 4, &dwRead, NULL);
				memcpy(glm::value_ptr(m4Temp), fMatrix, 16 * 4);

				ppPointij = (Pointij *)calloc(uiPointCount, sizeof(Pointij));

				if (ppPointij)
				{
					ReadFile(hFile, ppPointij, sizeof(Pointij) * uiPointCount, &dwRead, NULL);

					//Initiate Point Cloud Class
					pclPointCloud->clear();
					pclPointCloud->width = TANGO_IMAGE_WIDTH;
					pclPointCloud->height = TANGO_IMAGE_HEIGHT;
					pclPointCloud->resize(TANGO_IMAGE_WIDTH * TANGO_IMAGE_HEIGHT);

					pclPointCloudUnordered->clear();
					pclPointCloudUnordered->width = uiPointCount;
					pclPointCloudUnordered->height = 1;
					pclPointCloudUnordered->resize(uiPointCount);

					//Set NAN
					concurrency::parallel_for(size_t(0), (size_t)(TANGO_IMAGE_WIDTH * TANGO_IMAGE_HEIGHT), [&](size_t i)
					{
						pclPointCloud->at(i).x = NAN;
						pclPointCloud->at(i).y = NAN;
						pclPointCloud->at(i).z = NAN;
					});

					//Set Read Point Cloud
					concurrency::parallel_for(size_t(0), uiPointCount, [&](size_t i)
					{
						glm::vec4 v4Temp;
						size_t idx;
						Pointij *ppNow;

						ppNow = ppPointij + i;
						v4Temp.w = 1;
						idx = ppNow->ijs.x + ppNow->ijs.y * TANGO_IMAGE_WIDTH;

						memcpy(glm::value_ptr(v4Temp), ppNow, 12);
						v4Temp = m4Temp * kOpengGL_T_Depth * v4Temp;

						pclPointCloud->at(idx).x = v4Temp.x;
						pclPointCloud->at(idx).y = v4Temp.y;
						pclPointCloud->at(idx).z = v4Temp.z;
						pclPointCloud->at(idx).rgba = ppNow->c;

						pclPointCloudUnordered->at(i).x = v4Temp.x;
						pclPointCloudUnordered->at(i).y = v4Temp.y;
						pclPointCloudUnordered->at(i).z = v4Temp.z;
						pclPointCloudUnordered->at(i).rgba = ppNow->c;
					});

					free(ppPointij);
				}
			}

			CloseHandle(hFile);

			return TRUE;
		}

		return FALSE;
	}
}