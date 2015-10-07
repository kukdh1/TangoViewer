#include "PointCloud.h"

namespace kukdh1
{
	PointCloud::PointCloud()
	{
		pclPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	PointCloud::~PointCloud()
	{

	}

	void PointCloud::DrawOnGLWindow()
	{
		size_t stSize;

		glPushMatrix();

		glMultMatrixf(glm::value_ptr(m4Adjust));

		glPointSize(3.0f);
		glBegin(GL_POINTS);

		stSize = pclPointCloud->size();

		for (unsigned int i = 0; i < stSize; i++)
		{
			pcl::PointXYZRGB &pclPointXYZRGB = pclPointCloud->at(i);

			glColor3ub(pclPointXYZRGB.b, pclPointXYZRGB.g, pclPointXYZRGB.r);

			glVertex3f(pclPointXYZRGB.x, pclPointXYZRGB.y, pclPointXYZRGB.z);
		}

		glEnd();

		glPopMatrix();
	}

	void PointCloud::AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix)
	{
		m4Adjust *= m4AdjustMatrix;
	}

	void PointCloud::ApplyStaticalOutlierRemoveFilter()
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pclSOR;

		pclSOR.setInputCloud(pclPointCloud);
		pclSOR.setMeanK(SOR_MEAN_K);
		pclSOR.setStddevMulThresh(SOR_STDDEV_MUL_THRES);
		pclSOR.filter(*pclOutput);

		pclPointCloud = pclOutput;
	}
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB>>> pclRegion;

	void PointCloud::ExecutePlaneSegmentation()
	{
		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> pclOMPS;
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> pclNE;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr pclKdTree(new pcl::search::KdTree<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::Normal>::Ptr pclNormalCloud(new pcl::PointCloud<pcl::Normal>);

		//Estimate Normals
		pclNE.setInputCloud(pclPointCloud);
		pclNE.setSearchMethod(pclKdTree);
		pclNE.setRadiusSearch(NE_RADIUS_SEARCH);
		pclNE.compute(*pclNormalCloud);

		//Segment Planes
		pclOMPS.setMinInliers(OMPS_MINIMAL_INLIERS_PLANE);
		pclOMPS.setAngularThreshold(OMPS_ANGULAR_THRESHOLD);
		pclOMPS.setDistanceThreshold(OMPS_DISTANCE_THRESHOLD);
		pclOMPS.setInputNormals(pclNormalCloud);
		pclOMPS.setInputCloud(pclPointCloud);
		pclOMPS.segmentAndRefine(pclRegion);
	}

	BOOL PointCloud::FromFile(WCHAR * pszFilePath)
	{
		HANDLE hFile;
		DWORD dwRead;
		Point *ppPoints;
		glm::mat4 m4Temp;
		glm::vec4 v4Temp;
		pcl::PointXYZRGB pclPointXYZRGB;
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

				ppPoints = (Point *)calloc(uiPointCount, sizeof(Point));

				if (ppPoints)
				{
					ReadFile(hFile, ppPoints, 16 * uiPointCount, &dwRead, NULL);

					v4Temp.w = 1;

					for (unsigned int i = 0; i < uiPointCount; i++)
					{
						memcpy(glm::value_ptr(v4Temp), ppPoints + i, 12);
						v4Temp = m4Temp * kOpengGL_T_Depth * v4Temp;
						
						pclPointXYZRGB.x = v4Temp.x;
						pclPointXYZRGB.y = v4Temp.y;
						pclPointXYZRGB.z = v4Temp.z;
						pclPointXYZRGB.rgba = ppPoints[i].c;

						pclPointCloud->push_back(pcl::PointXYZRGB(pclPointXYZRGB));
					}
				}
			}

			CloseHandle(hFile);

			return TRUE;
		}

		return FALSE;
	}
}