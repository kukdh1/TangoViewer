#pragma once

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#define NOMINMAX

#include <Windows.h>
#include <gl/GL.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_inverse.hpp"

#define RADIAN_PER_DEGREE			0.017453f
#define OMPS_DISTANCE_THRESHOLD		0.02
#define OMPS_MINIMAL_INLIERS_PLANE	500
#define OMPS_ANGULAR_THRESHOLD		RADIAN_PER_DEGREE * 2

#define SOR_MEAN_K				50
#define SOR_STDDEV_MUL_THRES	1.0

#define NE_RADIUS_SEARCH		0.03


const glm::mat4 kOpengGL_T_Depth =
glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

namespace kukdh1
{
	/* Point struct
	 *  It contains 3D coordinate (XYZ) and its color (ARGB)
	 */
	struct Point
	{
		float x;
		float y;
		float z;
		COLORREF c;
	};

	/* PointCloud class
	 *  It contains PointCloud at such time
	 */
	class PointCloud
	{
		private:
			glm::mat4 m4Adjust;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud;

		public:
			PointCloud();
			~PointCloud();

			void DrawOnGLWindow();
			
			void AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix);
			void ApplyStaticalOutlierRemoveFilter();
			void ExecutePlaneSegmentation();

			BOOL FromFile(WCHAR *pszFilePath);
	};

}

#endif
