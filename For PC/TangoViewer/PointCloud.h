#pragma once

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#define NOMINMAX

#include <Windows.h>
#include <gl/GL.h>
#include <ppl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtx/vector_angle.hpp"

#include "Constant.h"
#include "File.h"

//wingdi.h:1846
#define RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

const glm::mat4 kOpengGL_T_Depth = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, -1.0f, 0.0f, 0.0f,
	0.0f, 0.0f,	-1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

const glm::mat4 kDepth_T_Axis = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, -1.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

const glm::mat4 kAxis_T_Depth = glm::mat4(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, -1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

namespace kukdh1
{
	glm::vec3 operator+(pcl::PointXYZRGB &lhs, const pcl::PointXYZRGB &rhs);

	/* Point struct
	 *  It contains 3D coordinate (XYZ) and its color (RGBA)
	 */
	struct Point
	{
		float x;
		float y;
		float z;
		COLORREF c;
	};

	/* Point struct
	*  It contains 3D coordinate (XYZ), its color (RGBA) and position (ij)
	*/
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

	/* BoundingBox struct
	 *  It contains 8 points of cube
	 */
	template <typename PointT>
	struct BoundingBox
	{
		PointT Point[8];
	};

	/* Plane Information struct
	 *  It contains plane information of point cloud
	 */
	struct PlaneInfo
	{
		pcl::ModelCoefficients::Ptr pclModelCoeff;
		Eigen::Vector3f pclCentroid;
		BoundingBox<pcl::PointXYZRGB> pclBoundingBox;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPoints;

		PlaneInfo()
		{
			pclModelCoeff = NULL;
			pclPoints = NULL;
		}
		~PlaneInfo()
		{
			if (pclModelCoeff)
				pclModelCoeff.reset();
			if (pclPoints)
				pclPoints.reset();
		}
	};

	/* Helper Functions
	 *  Point Cloud Helper Functions
	 */

	/* TransformPoint Function
	 *  Transform one point using translation nad rotation.
	 */
	template <typename PointT>
	void TransformPoint(PointT &input, PointT &translate, Eigen::Matrix3f &rotate);
	
	/* TransformPoint Function
	 *  Transform one point using 4by4 transform matrix.
	 */
	template <typename PointT>
	void TransformPoint(PointT &input, glm::mat4 &transform);
	void TransformPoint(Eigen::Vector3f &input, glm::mat4 &transform);

	/* TransformBoundingBox Function
	 *  Transform BoundingBox struct using translation and rotation.
	 */
	template <typename PointT>
	void TransformBoundingBox(BoundingBox<PointT> &input, pcl::PointXYZRGB &translate, Eigen::Matrix3f &rotate);

	/* TransformPlane Function
	 *  Transform plane using transform matrix.
	 */
	void TransformPlane(pcl::ModelCoefficients::Ptr pclPlane, glm::mat4 &transform);

	/* DrawCube Function
	 *  Draw a cube in GL Window from BoundingBox struct.
	 */
	void DrawCube(BoundingBox<pcl::PointXYZRGB> &pclRegion, COLORREF color);

	/* CalculateCenterOfBoundingBox Function
	 *  Calculate center of the bounding box
	 */
	glm::vec3 CalculateCenterOfBoundingBox(BoundingBox<pcl::PointXYZRGB> &pclRegion);

	/* PointCloud class
	 *  It contains PointCloud at such time
	 */
	class PointCloud
	{
		private:
			glm::mat4 m4Adjust;

			float fTimestamp;

			pcl::PointXYZRGB pclOrigin;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloudUnordered;
			BoundingBox<pcl::PointXYZRGB> pclBoundingBox;

			std::vector<PlaneInfo> pclPlaneInfo;

			BOOL bFiltered;
			BOOL bOBBCalculated;
			BOOL bPlaneCalculated;

		public:
			PointCloud();
			virtual ~PointCloud();

			void DrawOnGLWindow(BOOL bDrawOrigin, BOOL bDrawBoundingBox, BOOL bDrawPlanes);
			
			void AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix);
			void ResetTransformMatrix();
			void SaveTransformMatrix();
			void ApplyStaticalOutlierRemoveFilter();
			void CalculateBoundingBox();
			size_t ExecutePlaneSegmentation();

			static void * operator new(size_t);
			static void operator delete(void *);

			BOOL FromFile(WCHAR *pszFilePath);
			BOOL ToFile(WCHAR *pszFilePath);
			float GetTimestamp();

			BOOL GetFilterStatus();
			BOOL GetOBBStatus();
			BOOL GetPlaneSegmentationStatus();

			std::vector<PlaneInfo> & GetPlaneInfo();
			pcl::PointXYZRGB & GetPointCloudOrigin();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr & GetPointCloudData();
			BoundingBox<pcl::PointXYZRGB> & GetPointCloudOBB();
			glm::mat4 & GetAdjustMatrix();
	};

	typedef std::vector<PointCloud *> PointCloudVector;
}

#endif
