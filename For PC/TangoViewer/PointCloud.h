#pragma once

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#define NOMINMAX

#include <Windows.h>
#include <gl/GL.h>
#include <ppl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_inverse.hpp"

#define TANGO_IMAGE_WIDTH		1280
#define TANGO_IMAGE_HEIGHT		720

#define SAC_SEG_MAX_ITERATION			1000
#define SAC_SEG_DISTANCE_THRESHOLD		0.05
#define SAC_SEG_RATIO					0.1

#define RADIAN_PER_DEGREE			0.017453f
#define OMPS_DISTANCE_THRESHOLD		0.02
#define OMPS_MINIMAL_INLIERS_PLANE	2000
#define OMPS_ANGULAR_THRESHOLD		RADIAN_PER_DEGREE * 2
#define OMPS_MAXIMUM_CULVATURE		0x002

#define IINE_METHOD				AVERAGE_3D_GRADIENT
#define IINE_MAX_DEPTH_FACTOR	0.02f
#define IINE_SMOOTH_FACTOR		10.0f


#define SOR_MEAN_K				50
#define SOR_STDDEV_MUL_THRES	1.0


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

	struct Pointij
	{
		float x;
		float y;
		float z;
		COLORREF c;
		union
		{
			unsigned int ij;
			struct {
				unsigned short x;
				unsigned short y;
			};
		} ijs;
	};

	/* PointCloud class
	 *  It contains PointCloud at such time
	 */
	class PointCloud
	{
		private:
			glm::mat4 m4Adjust;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloudUnordered;
		//	std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB>>> pclRegion;
			std::vector<pcl::ModelCoefficients::Ptr> pclPlaneCoeffs;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pclPlanePoints;

		public:
			PointCloud();
			~PointCloud();

			void DrawOnGLWindow(BOOL bDrawPlanes);
			
			void AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix);
			void ApplyStaticalOutlierRemoveFilter();
			size_t ExecutePlaneSegmentation();

			static void * operator new(size_t);
			static void operator delete(void *);

			BOOL FromFile(WCHAR *pszFilePath);
	};

}

#endif
