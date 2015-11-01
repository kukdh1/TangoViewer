#include "LUM.h"

namespace kukdh1
{
	Eigen::Vector6f LUM::ConvertTransformMatrix(glm::mat4 &m4Transform)
	{
		Eigen::Vector6f eigenReturn;
		glm::vec3 scale;
		glm::quat rotation;
		glm::vec3 translation;
		glm::vec3 skew;
		glm::vec4 perspective;

		glm::decompose(m4Transform, scale, rotation, translation, skew, perspective);

		eigenReturn << translation.x, translation.y, translation.z, glm::roll(rotation), glm::pitch(rotation), glm::yaw(rotation);

		return eigenReturn;
	}

	pcl::CorrespondencesPtr LUM::CalculateCorrespondence(PointCloud &target, PointCloud &input)
	{
		pcl::CorrespondencesPtr pclCorr;
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> pclCE;
		
		pclCorr = pcl::CorrespondencesPtr(new pcl::Correspondences);
		pclCE.setInputTarget(target.GetPointCloudData());
		pclCE.setInputSource(input.GetPointCloudData());
		pclCE.determineCorrespondences(*pclCorr, CORR_MAX_DISTANCE);

		return pclCorr;
	}

	void LUM::AddPointClouds(PointCloudVector &input)
	{
		size_t stInputCount;
		size_t stVertex;
		glm::mat4 m4First;

		stInputCount = input.size();

		if (stInputCount == 0)
			return;
		
		m4First = glm::inverse(input.at(0)->GetAdjustMatrix());

		stVertex = pclLUM.addPointCloud(input.at(0)->GetPointCloudData());
		pVertexIndex.push_back(std::pair<size_t, size_t>(0, stVertex));
		
		if (stInputCount > 1)
		{
			for (size_t i = 1; i < stInputCount; i++)
			{
				glm::mat4 m4Transform;

				m4Transform = input.at(i)->GetAdjustMatrix() * m4First;
				stVertex = pclLUM.addPointCloud(input.at(i)->GetPointCloudData());// , ConvertTransformMatrix(m4Transform));
				pVertexIndex.push_back(std::pair<size_t, size_t>(i, stVertex));
			}

			for (size_t i = 1; i < stInputCount; i++)
			{
				glm::vec3 v3CentroidI;

				v3CentroidI = CalculateCenterOfBoundingBox(input.at(i)->GetPointCloudOBB());

				for (size_t j = 0; j < i; j++)
				{
					glm::vec3 v3CentroidJ;

					v3CentroidJ = CalculateCenterOfBoundingBox(input.at(j)->GetPointCloudOBB());

					if ((v3CentroidI - v3CentroidJ).length() < CENTROID_MAX_DISTANCE)
					{
						pcl::CorrespondencesPtr pclCorr;

						pclCorr = CalculateCorrespondence(*input.at(i), *input.at(j));

						if (pclCorr->size() > 2)
							pclLUM.setCorrespondences(j, i, pclCorr);
					}
				}
			}
		}

		pcvData = &input;
	}

	glm::mat4 convert(Eigen::Matrix4f &input)
	{
		glm::mat4 m4Return;

		memcpy(glm::value_ptr(m4Return), input.data(), sizeof(float) * 16);
		
		return m4Return;
	}

	void LUM::Compute()
	{
		size_t stInputCount;

		pclLUM.setMaxIterations(LUM_MAX_ITERATION);
		pclLUM.compute();

		stInputCount = pclLUM.getNumVertices();

		for (size_t i = 0; i < stInputCount; i++)
		{
			Eigen::Affine3f eigenTransform;
			glm::mat4 m4Trnasform;

			eigenTransform = pclLUM.getTransformation(i);
			m4Trnasform = convert(eigenTransform.matrix());

			for (size_t j = 0; j < stInputCount; j++)
				if (pVertexIndex.at(j).second == i)
					pcvData->at(pVertexIndex.at(j).first)->AdjustTransformMatrix(m4Trnasform);
		}
	}
}