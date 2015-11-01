#pragma once

#ifndef _LUM_H_
#define _LUM_H_

#define NOMINMAX

#include <Windows.h>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>

#include "glm/gtx/matrix_decompose.hpp"

#include "PointCloud.h"
#include "Constant.h"

namespace kukdh1
{
	class LUM
	{
		private:
			pcl::registration::LUM<pcl::PointXYZRGB> pclLUM;

			Eigen::Vector6f ConvertTransformMatrix(glm::mat4 &m4Transform);
			pcl::CorrespondencesPtr CalculateCorrespondence(PointCloud &target, PointCloud &input);
			
			std::vector<std::pair<size_t, size_t>> pVertexIndex;
			PointCloudVector *pcvData;

		public:
			void AddPointClouds(PointCloudVector &input);
			void Compute();
	};
}

#endif