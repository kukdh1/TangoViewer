#pragma once

#ifndef _POINT_CLOUD_MERGE_H_
#define _POINT_CLOUD_MERGE_H_

#define NOMINMAX

#include <Windows.h>
#include <assert.h>

#include "PointCloud.h"
#include "Constant.h"
#include "Log.h"
#include "Stopwatch.h"

namespace kukdh1
{
	struct PLANEPAIR{
		size_t idxA;
		size_t idxB;
		BOOL bAngleCorrectionNeed;
		BOOL bDistanceCorrectionNeed;
	};

	enum MERGE_RESULT{
		MERGE_OK,
		MERGE_OK_NO_OPERATION,
		MERGE_NO_INTERSECTION,
		MERGE_CANNOT_CALCULATE,
		MERGE_UNKNOWN,
	};

	/* CalculateAngleBetweenPlanes Function
	*   Calculate angle between two planes using normals.
	*/
	float CalculateAngleBetweenPlanes(pcl::ModelCoefficients::Ptr pclPlaneA, pcl::ModelCoefficients::Ptr pclPlaneB);

	/* CalculateAngleBetweenVectors Function
	 *  Calculate angle between two vector
	 */
	float CalculateAngleBetweenVectors(glm::vec3 v3PlaneA, glm::vec3 v3PlaneB);

	/* CalculateDistanceBetweenPlanes Function
	*  Calculate distance between two planes.
	*/
	float CalculateDistanceBetweenPlanes(pcl::ModelCoefficients::Ptr pclPlane, Eigen::Vector3f &ptPoint);

	/* IsIntersect Function
	*  Check two bounding box whether intersect.
	*  Return false with no intersection.
	*/
	template <typename PointT>
	BOOL IsIntersect(BoundingBox<PointT> &inputA, BoundingBox<PointT> &inputB);

	/* CalculateCorrection Function
	 *  Error Correction Function. It calculates transform matrix of inputB.
	 */
	MERGE_RESULT CalculateCorrection(PointCloud &inputA, PointCloud &inputB);

	/* PlaneTree Class
	 *  Make relation graph for correction
	 */
	class PlaneTree
	{
		private:
			std::vector<std::vector<int> *> vTreeData;
			std::vector<PointCloud *> *pvCloudData;

		public:
			~PlaneTree();

			void AddPointCloud(std::vector<PointCloud *> &vCloudData);
			void DoErrorCorrection(LogWindow *pLogWindow);
	};
}

#endif