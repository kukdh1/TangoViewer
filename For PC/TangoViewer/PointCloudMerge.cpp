#include "PointCloudMerge.h"

namespace kukdh1
{
	glm::vec3 operator-(pcl::PointXYZRGB &lhs, const pcl::PointXYZRGB &rhs)
	{
		return glm::vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
	}

	glm::vec3 convert(pcl::PointXYZRGB &point)
	{
		return glm::vec3(point.x, point.y, point.z);
	}

	glm::vec3 convert(pcl::ModelCoefficients::Ptr coeff)
	{
		return glm::vec3(coeff->values[0], coeff->values[1], coeff->values[2]);
	}

	glm::vec3 convert(Eigen::Vector3f &point)
	{
		return glm::vec3(point(0), point(1), point(2));
	}

	template<typename PointT>
	BOOL IsIntersect(BoundingBox<PointT>& inputA, BoundingBox<PointT>& inputB)
	{
		//http://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrOrientedBox3OrientedBox3.h

		glm::vec3 v3ACenter;
		glm::vec3 v3AAxis[3];
		float v3AExtent[3];
		glm::vec3 v3BCenter;
		glm::vec3 v3BAxis[3];
		float v3BExtent[3];
		glm::vec3 v3Distance0;
		float cutoff;
		bool existsParallelPair;

		cutoff = 1;
		existsParallelPair = false;

		v3AAxis[0] = glm::normalize(inputA.Point[3] - inputA.Point[0]);
		v3AAxis[1] = glm::normalize(inputA.Point[1] - inputA.Point[0]);
		v3AAxis[2] = glm::normalize(inputA.Point[4] - inputA.Point[0]);

		v3BAxis[0] = glm::normalize(inputB.Point[3] - inputB.Point[0]);
		v3BAxis[1] = glm::normalize(inputB.Point[1] - inputB.Point[0]);
		v3BAxis[2] = glm::normalize(inputB.Point[4] - inputB.Point[0]);

		v3AExtent[0] = glm::distance(convert(inputA.Point[3]), convert(inputA.Point[0])) / 2;
		v3AExtent[1] = glm::distance(convert(inputA.Point[1]), convert(inputA.Point[0])) / 2;
		v3AExtent[2] = glm::distance(convert(inputA.Point[4]), convert(inputA.Point[0])) / 2;

		v3BExtent[0] = glm::distance(convert(inputB.Point[3]), convert(inputB.Point[0])) / 2;
		v3BExtent[1] = glm::distance(convert(inputB.Point[1]), convert(inputB.Point[0])) / 2;
		v3BExtent[2] = glm::distance(convert(inputB.Point[4]), convert(inputB.Point[0])) / 2;

		v3ACenter = (inputA.Point[0] + inputA.Point[6]) / 2.f;
		v3BCenter = (inputB.Point[0] + inputB.Point[6]) / 2.f;

		// Compute difference of box centers.
		glm::vec3 D = v3BCenter - v3ACenter;

		float dot01[3][3];       // dot01[i][j] = Dot(A0[i],A1[j]) = A1[j][i]
		float absDot01[3][3];    // |dot01[i][j]|
		float dotDA0[3];         // Dot(D, A0[i])
		float r0, r1, r;         // interval radii and distance between centers
		float r01;               // r0 + r1

								 // Test for separation on the axis C0 + t*A0[0].
		for (int i = 0; i < 3; ++i)
		{
			dot01[0][i] = glm::dot(v3AAxis[0], v3BAxis[i]);
			absDot01[0][i] = fabsf(dot01[0][i]);
			if (absDot01[0][i] > cutoff)
			{
				existsParallelPair = true;
			}
		}

		dotDA0[0] = glm::dot(D, v3AAxis[0]);
		r = fabsf(dotDA0[0]);
		r1 = v3BExtent[0] * absDot01[0][0] + v3BExtent[1] * absDot01[0][1] + v3BExtent[2] * absDot01[0][2];
		r01 = v3AExtent[0] + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[1].
		for (int i = 0; i < 3; ++i)
		{
			dot01[1][i] = glm::dot(v3AAxis[1], v3BAxis[i]);
			absDot01[1][i] = fabsf(dot01[1][i]);
			if (absDot01[1][i] > cutoff)
			{
				existsParallelPair = true;
			}
		}

		dotDA0[1] = glm::dot(D, v3AAxis[1]);
		r = fabsf(dotDA0[1]);
		r1 = v3BExtent[0] * absDot01[1][0] + v3BExtent[1] * absDot01[1][1] + v3BExtent[2] * absDot01[1][2];
		r01 = v3AExtent[1] + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[2].
		for (int i = 0; i < 3; ++i)
		{
			dot01[2][i] = glm::dot(v3AAxis[2], v3BAxis[i]);
			absDot01[2][i] = fabsf(dot01[2][i]);
			if (absDot01[2][i] > cutoff)
			{
				existsParallelPair = true;
			}
		}

		dotDA0[2] = glm::dot(D, v3AAxis[2]);
		r = fabsf(dotDA0[2]);
		r1 = v3BExtent[0] * absDot01[2][0] + v3BExtent[1] * absDot01[2][1] + v3BExtent[2] * absDot01[2][2];
		r01 = v3AExtent[2] + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A1[0].
		r = fabsf(glm::dot(D, v3BAxis[0]));
		r0 = v3AExtent[0] * absDot01[0][0] + v3AExtent[1] * absDot01[1][0] + v3AExtent[2] * absDot01[2][0];
		r01 = r0 + v3BExtent[0];
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A1[1].
		r = fabsf(glm::dot(D, v3BAxis[1]));
		r0 = v3AExtent[0] * absDot01[0][1] + v3AExtent[1] * absDot01[1][1] + v3AExtent[2] * absDot01[2][1];
		r01 = r0 + v3BExtent[1];
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A1[2].
		r = fabsf(glm::dot(D, v3BAxis[2]));
		r0 = v3AExtent[0] * absDot01[0][2] + v3AExtent[1] * absDot01[1][2] + v3AExtent[2] * absDot01[2][2];
		r01 = r0 + v3BExtent[2];
		if (r > r01)
			return FALSE;

		// At least one pair of box axes was parallel, so the separation is
		// effectively in 2D.  The edge-edge axes do not need to be tested.
		if (existsParallelPair)
			return TRUE;

		// Test for separation on the axis C0 + t*A0[0]xA1[0].
		r = fabsf(dotDA0[2] * dot01[1][0] - dotDA0[1] * dot01[2][0]);
		r0 = v3AExtent[1] * absDot01[2][0] + v3AExtent[2] * absDot01[1][0];
		r1 = v3BExtent[1] * absDot01[0][2] + v3BExtent[2] * absDot01[0][1];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[0]xA1[1].
		r = fabsf(dotDA0[2] * dot01[1][1] - dotDA0[1] * dot01[2][1]);
		r0 = v3AExtent[1] * absDot01[2][1] + v3AExtent[2] * absDot01[1][1];
		r1 = v3BExtent[0] * absDot01[0][2] + v3BExtent[2] * absDot01[0][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[0]xA1[2].
		r = fabsf(dotDA0[2] * dot01[1][2] - dotDA0[1] * dot01[2][2]);
		r0 = v3AExtent[1] * absDot01[2][2] + v3AExtent[2] * absDot01[1][2];
		r1 = v3BExtent[0] * absDot01[0][1] + v3BExtent[1] * absDot01[0][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[1]xA1[0].
		r = fabsf(dotDA0[0] * dot01[2][0] - dotDA0[2] * dot01[0][0]);
		r0 = v3AExtent[0] * absDot01[2][0] + v3AExtent[2] * absDot01[0][0];
		r1 = v3BExtent[1] * absDot01[1][2] + v3BExtent[2] * absDot01[1][1];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[1]xA1[1].
		r = fabsf(dotDA0[0] * dot01[2][1] - dotDA0[2] * dot01[0][1]);
		r0 = v3AExtent[0] * absDot01[2][1] + v3AExtent[2] * absDot01[0][1];
		r1 = v3BExtent[0] * absDot01[1][2] + v3BExtent[2] * absDot01[1][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[1]xA1[2].
		r = fabsf(dotDA0[0] * dot01[2][2] - dotDA0[2] * dot01[0][2]);
		r0 = v3AExtent[0] * absDot01[2][2] + v3AExtent[2] * absDot01[0][2];
		r1 = v3BExtent[0] * absDot01[1][1] + v3BExtent[1] * absDot01[1][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[2]xA1[0].
		r = fabsf(dotDA0[1] * dot01[0][0] - dotDA0[0] * dot01[1][0]);
		r0 = v3AExtent[0] * absDot01[1][0] + v3AExtent[1] * absDot01[0][0];
		r1 = v3BExtent[1] * absDot01[2][2] + v3BExtent[2] * absDot01[2][1];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[2]xA1[1].
		r = fabsf(dotDA0[1] * dot01[0][1] - dotDA0[0] * dot01[1][1]);
		r0 = v3AExtent[0] * absDot01[1][1] + v3AExtent[1] * absDot01[0][1];
		r1 = v3BExtent[0] * absDot01[2][2] + v3BExtent[2] * absDot01[2][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		// Test for separation on the axis C0 + t*A0[2]xA1[2].
		r = fabsf(dotDA0[1] * dot01[0][2] - dotDA0[0] * dot01[1][2]);
		r0 = v3AExtent[0] * absDot01[1][2] + v3AExtent[1] * absDot01[0][2];
		r1 = v3BExtent[0] * absDot01[2][1] + v3BExtent[1] * absDot01[2][0];
		r01 = r0 + r1;
		if (r > r01)
			return FALSE;

		return TRUE;
	}

	float CalculateAngleBetweenPlanes(pcl::ModelCoefficients::Ptr pclPlaneA, pcl::ModelCoefficients::Ptr pclPlaneB)
	{
		glm::vec3 v3PlaneA;
		glm::vec3 v3PlaneB;

		v3PlaneA.x = pclPlaneA->values[0];
		v3PlaneA.y = pclPlaneA->values[1];
		v3PlaneA.z = pclPlaneA->values[2];

		v3PlaneB.x = pclPlaneB->values[0];
		v3PlaneB.y = pclPlaneB->values[1];
		v3PlaneB.z = pclPlaneB->values[2];

		return CalculateAngleBetweenVectors(v3PlaneA, v3PlaneB);
	}

	float CalculateAngleBetweenVectors2(glm::vec3 &v3PlaneA, glm::vec3 &v3PlaneB)
	{
		v3PlaneA = glm::normalize(v3PlaneA);
		v3PlaneB = glm::normalize(v3PlaneB);

		return glm::angle(v3PlaneA, v3PlaneB);
	}

	float CalculateAngleBetweenVectors(glm::vec3 v3PlaneA, glm::vec3 v3PlaneB)
	{
		float fResult;

		fResult = CalculateAngleBetweenVectors2(v3PlaneA, v3PlaneB);

		return fResult > PI_2 ? PI - fResult : fResult;
	}

	float CalculateDistanceBetweenPlanes(pcl::ModelCoefficients::Ptr pclPlane, Eigen::Vector3f &ptPoint)
	{
		glm::vec3 v3Plane;
		float fPlaneDistance;

		v3Plane.x = pclPlane->values[0];
		v3Plane.y = pclPlane->values[1];
		v3Plane.z = pclPlane->values[2];

		fPlaneDistance = fabsf(pclPlane->values[0] * ptPoint(0) + pclPlane->values[1] * ptPoint(1) + pclPlane->values[2] * ptPoint(2) + pclPlane->values[3]) / glm::length(v3Plane);

		return fPlaneDistance;
	}

	glm::vec3 CalculateSumOfNormals(pcl::ModelCoefficients::Ptr inputA, pcl::ModelCoefficients::Ptr inputB)
	{
		glm::vec3 v3A;
		glm::vec3 v3B;

		v3A.x = inputA->values[0];
		v3A.y = inputA->values[1];
		v3A.z = inputA->values[2];

		v3B.x = inputB->values[0];
		v3B.y = inputB->values[1];
		v3B.z = inputB->values[2];

		if (CalculateAngleBetweenVectors2(v3A, v3B) > PI_2)
			v3A *= -1.f;

		return glm::normalize(v3A + v3B);
	}

	glm::vec3 CalculateIntersectionOfPlaneAndLine(pcl::ModelCoefficients::Ptr pclPlane, glm::vec3 v3LineDirection, glm::vec3 v3LinePoint)
	{
		float t;

		t = -1 * (pclPlane->values[0] * v3LinePoint.x + pclPlane->values[1] * v3LinePoint.y + pclPlane->values[2] * v3LinePoint.z + pclPlane->values[3])
			/ (pclPlane->values[0] * v3LineDirection.x + pclPlane->values[1] * v3LineDirection.y + pclPlane->values[2] * v3LineDirection.z);

		return v3LinePoint + t * v3LineDirection;
	}

	void UpdatePlainPairDistance(std::vector<PLANEPAIR> &vPlanePairC, std::vector<PlaneInfo> &piPlaneA, std::vector<PlaneInfo> &piPlaneB, glm::mat4 &m4TransformA, glm::mat4 &m4TransformB)
	{
		size_t stPlaneC;

		stPlaneC = vPlanePairC.size();

		for (size_t i = 0; i < stPlaneC; i++)
		{
			float fDistance;

			pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairC.at(i).idxA).pclModelCoeff));
			pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairC.at(i).idxB).pclModelCoeff));
			Eigen::Vector3f pclCentroidA = piPlaneA.at(vPlanePairC.at(i).idxA).pclCentroid;
			Eigen::Vector3f pclCentroidB = piPlaneB.at(vPlanePairC.at(i).idxB).pclCentroid;

			TransformPlane(pclPlaneA, m4TransformA);
			TransformPlane(pclPlaneB, m4TransformB);
			TransformPoint(pclCentroidA, m4TransformA);
			TransformPoint(pclCentroidB, m4TransformB);

			fDistance = CalculateDistanceBetweenPlanes(pclPlaneA, pclCentroidB) + CalculateDistanceBetweenPlanes(pclPlaneB, pclCentroidA);
			fDistance /= 2;

			vPlanePairC.at(i).bAngleCorrectionNeed = FALSE;
			if (fDistance < DISTANCE_THRESHOLD_SAME_PLANE)
				vPlanePairC.at(i).bDistanceCorrectionNeed = FALSE;
			else if (fDistance > DISTANCE_THRESHOLD_DEFFERENT_PLANE)
				vPlanePairC.at(i).bDistanceCorrectionNeed = 2;
			else
				vPlanePairC.at(i).bAngleCorrectionNeed = TRUE;
		}
	}

	void EnforcePreprocess(PointCloud &input)
	{
		if (!input.GetOBBStatus())
		{
			if (!input.GetPlaneSegmentationStatus())
			{
				if (!input.GetFilterStatus())
					input.ApplyStaticalOutlierRemoveFilter();

				input.ExecutePlaneSegmentation();
			}

			input.CalculateBoundingBox();
		}
	}

	MERGE_RESULT CalculateCorrection(PointCloud &inputA, PointCloud &inputB)
	{
		MERGE_RESULT mrResult = MERGE_UNKNOWN;

		//Check Filter/Plane/OBB
		EnforcePreprocess(inputA);
		EnforcePreprocess(inputB);

		/* Get Data
		 *  inputA is criterion of error correction.
		 *  inputB will be transformed by transform matrix.
		 */
		glm::vec3 vOriginA;
		glm::vec3 vOriginB;
		std::vector<PlaneInfo> &piPlaneA = inputA.GetPlaneInfo();
		std::vector<PlaneInfo> &piPlaneB = inputB.GetPlaneInfo();

		vOriginA = convert(inputA.GetPointCloudOrigin());
		vOriginB = convert(inputB.GetPointCloudOrigin());

		//Check Two input has intersection
		if (!IsIntersect(inputA.GetPointCloudOBB(), inputB.GetPointCloudOBB()))
			return MERGE_NO_INTERSECTION;

		/* Determine Plane
		 *  Group Planes into:
		 *   1. Different Plane
		 *   2. Same Plane but correction need
		 *   3. Same Plane without correction
		 */
		size_t stPlaneCountA;
		size_t stPlaneCountB;
		std::vector<PLANEPAIR> vPlanePairNC;
		std::vector<PLANEPAIR> vPlanePairC;
		glm::mat4 m4OriginalAdjustA;
		glm::mat4 m4OriginalAdjustB;

		m4OriginalAdjustA = inputA.GetAdjustMatrix();
		m4OriginalAdjustB = inputB.GetAdjustMatrix();

		stPlaneCountA = piPlaneA.size();
		stPlaneCountB = piPlaneB.size();

		for (size_t idxA = 0; idxA < stPlaneCountA; idxA++)
		{
			pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(idxA).pclModelCoeff));
			Eigen::Vector3f pclCentroidA = piPlaneA.at(idxA).pclCentroid;

			TransformPoint(pclCentroidA, m4OriginalAdjustA);
			TransformPlane(pclPlaneA, m4OriginalAdjustA);

			for (size_t idxB = 0; idxB < stPlaneCountB; idxB++)
			{
				PLANEPAIR ppTemp;
				float fAngle, fDistance;

				pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(idxB).pclModelCoeff));
				Eigen::Vector3f pclCentroidB = piPlaneB.at(idxB).pclCentroid;

				TransformPoint(pclCentroidB, m4OriginalAdjustB);
				TransformPlane(pclPlaneB, m4OriginalAdjustB);

				//Check Angle between two plane
				fAngle = CalculateAngleBetweenPlanes(pclPlaneA, pclPlaneB);

				if (fAngle < ANGLE_THRESHOLD_DIFFRENT_PLANE)
				{
					//Check Distance between two plane
					fDistance = CalculateDistanceBetweenPlanes(pclPlaneA, pclCentroidB)
						+ CalculateDistanceBetweenPlanes(pclPlaneB, pclCentroidA);
					fDistance /= 2;

					if (fDistance < DISTANCE_THRESHOLD_DEFFERENT_PLANE)
					{
						ppTemp.idxA = idxA;
						ppTemp.idxB = idxB;
						ppTemp.bAngleCorrectionNeed = TRUE;
						ppTemp.bDistanceCorrectionNeed = TRUE;

						if (fAngle <= ANGLE_THRESHOLD_SAME_PLANE)
							ppTemp.bAngleCorrectionNeed = FALSE;
						if (fDistance <= DISTANCE_THRESHOLD_SAME_PLANE)
							ppTemp.bDistanceCorrectionNeed = FALSE;

						if (ppTemp.bAngleCorrectionNeed || ppTemp.bDistanceCorrectionNeed)
							vPlanePairC.push_back(ppTemp);
						else
							vPlanePairNC.push_back(ppTemp);
					}
				}
			}
		}

		/* Normal Calculation
		 *  In the group of '3. Same Plane without correction', 
		 *  Calculate Normal Vectors.
		 */
		size_t stPlanePair;
		std::vector<glm::vec3> vPlaneVectors;

		stPlanePair = vPlanePairNC.size();
		for (size_t i = 0; i < stPlanePair; i++)
		{
			glm::vec3 v3Plane;
			pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairNC.at(i).idxA).pclModelCoeff));
			pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairNC.at(i).idxB).pclModelCoeff));

			TransformPlane(pclPlaneA, m4OriginalAdjustA);
			TransformPlane(pclPlaneB, m4OriginalAdjustB);

			v3Plane = CalculateSumOfNormals(pclPlaneA, pclPlaneB);

			vPlaneVectors.push_back(glm::normalize(v3Plane));
		}

		/* Normal Filtering
		 *  Erase 'almost same' normal vectors.
		 */
		std::vector<glm::vec3> vUniquePlaneVectors;

		stPlanePair = vPlaneVectors.size();
		for (size_t i = 0; i < stPlanePair; i++)
		{
			BOOL isSame = FALSE;

			for (size_t j = i + 1; j < stPlanePair; j++)
				if (CalculateAngleBetweenVectors(vPlaneVectors.at(i), vPlaneVectors.at(j)) < ANGLE_THRESHOLD_SAME_PLANE)
				{
					isSame = TRUE;
					break;
				}

			if (!isSame)
				vUniquePlaneVectors.push_back(vPlaneVectors.at(i));
		}

		/* Correction using normal vectors
		 *  Correction using FIXED normal vectors.
		 *  Data must rotate by fixed normal vectors.
		 *  Data must translate by perpendicular to fixed normal vectors.
		 */
		size_t stUniqueVector;

		stUniqueVector = vUniquePlaneVectors.size();

		if (stUniqueVector == 0)
		{
			mrResult = MERGE_CANNOT_CALCULATE;
		}
		else if (stUniqueVector == 1)
		{
			/* There is one unique normal vector.
			 * Point Cloud only can rotate by normal vector.
			 * Point Cloud can translate on the plane defined by normal vector.
			 */

			/* Rotational Correction
			 * Move Point Cloud Origin to Axis Origin.
			 * Calculate oriented angle using fixed normal vector.
			 * Make rotate matrix and apply.
			 */
			size_t stPlaneC;
			glm::mat4 m4Transform;
			float angle;

			stPlaneC = vPlanePairC.size();
			angle = 0.f;

			for (size_t i = 0; i < stPlaneC; i++)
				if (vPlanePairC.at(i).bAngleCorrectionNeed)
				{
					glm::vec3 v3Normal;
					pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairC.at(i).idxA).pclModelCoeff));
					pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairC.at(i).idxB).pclModelCoeff));

					TransformPlane(pclPlaneA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4OriginalAdjustB);

					v3Normal = CalculateSumOfNormals(pclPlaneA, pclPlaneB);

					if (CalculateAngleBetweenVectors(v3Normal, vUniquePlaneVectors.at(0)) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;

					angle += glm::orientedAngle(convert(piPlaneA.at(vPlanePairC.at(i).idxA).pclModelCoeff), convert(piPlaneB.at(vPlanePairC.at(i).idxB).pclModelCoeff), vUniquePlaneVectors.at(0));

					m4Transform = glm::translate(m4Transform, vOriginB);
					m4Transform = glm::rotate(m4Transform, angle, vUniquePlaneVectors.at(0));
					m4Transform = glm::translate(m4Transform, vOriginB * -1.f);

					//Check Direction of rotate
					TransformPlane(pclPlaneB, m4Transform);

					if (CalculateAngleBetweenPlanes(pclPlaneA, pclPlaneB) > ANGLE_THRESHOLD_SAME_PLANE)
					{
						m4Transform = glm::translate(glm::mat4(1.f), vOriginB);
						m4Transform = glm::rotate(m4Transform, -angle, vUniquePlaneVectors.at(0));
						m4Transform = glm::translate(m4Transform, vOriginB * -1.f);
					}

					break;
				}

			//Update Plane Pair
			UpdatePlainPairDistance(vPlanePairC, piPlaneA, piPlaneB, m4OriginalAdjustA, m4Transform * m4OriginalAdjustB);

			/* Translational Correction
			 * Find First correction vector.
			 * Correct error with correction vector.
			 * Calculate a vector perpendicular with correction vector and unique normal vector
			 * Correct error with new vector.
			 */
			glm::mat4 m4Transform2;
			size_t lasti;
			glm::vec3 v3FirstTranslate;

			for (lasti = 0; lasti < stPlaneC; lasti++)
			{
				if (vPlanePairC.at(lasti).bDistanceCorrectionNeed == TRUE)
				{
					glm::vec3 vNormal;
					pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairC.at(lasti).idxA).pclModelCoeff));
					pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairC.at(lasti).idxB).pclModelCoeff));
					Eigen::Vector3f pclCentroidA = piPlaneA.at(vPlanePairC.at(lasti).idxA).pclCentroid;

					/* Calculate direction of translation
					 *  Cross product of 'Unique normal vector' and 'perpendicular vector of unique vector and plane normal vector'
					 */
					TransformPlane(pclPlaneA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4OriginalAdjustB);
					TransformPoint(pclCentroidA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4Transform);

					vNormal = CalculateSumOfNormals(pclPlaneA, pclPlaneB);

					if (CalculateAngleBetweenVectors(vNormal, vUniquePlaneVectors.at(0)) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;

					v3FirstTranslate = glm::cross(glm::cross(vNormal, vUniquePlaneVectors.at(0)), vUniquePlaneVectors.at(0));

					glm::vec3 v3InterA;
					glm::vec3 v3InterB;

					v3InterA = CalculateIntersectionOfPlaneAndLine(pclPlaneA, v3FirstTranslate, convert(pclCentroidA));
					v3InterB = CalculateIntersectionOfPlaneAndLine(pclPlaneB, v3FirstTranslate, convert(pclCentroidA));

					m4Transform2 = glm::translate(m4Transform2, v3InterA - v3InterB);

					break;
				}
			}

			//Update Plane Pair
			UpdatePlainPairDistance(vPlanePairC, piPlaneA, piPlaneB, m4OriginalAdjustA, m4Transform2 * m4Transform * m4OriginalAdjustB);

			glm::vec3 v3SecondTranslate;
			
			v3SecondTranslate = glm::cross(v3FirstTranslate, vUniquePlaneVectors.at(0));

			for (size_t i = lasti + 1; i < stPlaneC; i++)
			{
				if (vPlanePairC.at(lasti).bDistanceCorrectionNeed == TRUE)
				{
					glm::vec3 vNormal;
					pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairC.at(lasti).idxA).pclModelCoeff));
					pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairC.at(lasti).idxB).pclModelCoeff));
					Eigen::Vector3f pclCentroidA = piPlaneA.at(vPlanePairC.at(lasti).idxA).pclCentroid;

					/* Calculate direction of translation
					*  Cross product of 'Unique normal vector' and 'perpendicular vector of unique vector and plane normal vector'
					*/
					TransformPlane(pclPlaneA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4OriginalAdjustB);
					TransformPoint(pclCentroidA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4Transform);
					TransformPlane(pclPlaneB, m4Transform2);

					vNormal = CalculateSumOfNormals(pclPlaneA, pclPlaneB);

					if (CalculateAngleBetweenVectors(vNormal, v3FirstTranslate) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;
					if (CalculateAngleBetweenVectors(vNormal, vUniquePlaneVectors.at(0)) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;

					glm::vec3 v3InterA;
					glm::vec3 v3InterB;

					v3InterA = CalculateIntersectionOfPlaneAndLine(pclPlaneA, v3SecondTranslate, convert(pclCentroidA));
					v3InterB = CalculateIntersectionOfPlaneAndLine(pclPlaneB, v3SecondTranslate, convert(pclCentroidA));

					m4Transform2 = glm::translate(m4Transform2, v3InterA - v3InterB);

					break;
				}
			}

			//Apply Result
			inputB.AdjustTransformMatrix(m4Transform2 * m4Transform);

			mrResult = MERGE_OK;
		}
		else if (stUniqueVector == 2)
		{
			/* There is two unique normal vectors.
			 * Point Cloud cannot rotate and only translatable to one direction which
			 * perpendicular to two unique normal vectors.
			 */

			 /* Translational Correction
			 * Find correction vector.
			 * Correct error with correction vector.
			 */
			size_t stPlaneC;
			glm::vec3 v3Translate;
			glm::mat4 m4Transform;

			stPlaneC = vPlanePairC.size();

			v3Translate = glm::cross(vUniquePlaneVectors.at(0), vUniquePlaneVectors.at(1));

			for (size_t i = 0; i < stPlaneC; i++)
			{
				if (vPlanePairC.at(i).bDistanceCorrectionNeed == TRUE)
				{
					glm::vec3 vNormal;
					pcl::ModelCoefficients::Ptr pclPlaneA = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneA.at(vPlanePairC.at(i).idxA).pclModelCoeff));
					pcl::ModelCoefficients::Ptr pclPlaneB = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*piPlaneB.at(vPlanePairC.at(i).idxB).pclModelCoeff));
					Eigen::Vector3f pclCentroidA = piPlaneA.at(vPlanePairC.at(i).idxA).pclCentroid;

					TransformPlane(pclPlaneA, m4OriginalAdjustA);
					TransformPlane(pclPlaneB, m4OriginalAdjustB);
					TransformPoint(pclCentroidA, m4OriginalAdjustA);

					vNormal = CalculateSumOfNormals(pclPlaneA, pclPlaneB);

					if (CalculateAngleBetweenVectors(vNormal, vUniquePlaneVectors.at(0)) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;
					if (CalculateAngleBetweenVectors(vNormal, vUniquePlaneVectors.at(1)) < ANGLE_THRESHOLD_DIFFRENT_PLANE)
						continue;

					glm::vec3 v3InterA;
					glm::vec3 v3InterB;

					v3InterA = CalculateIntersectionOfPlaneAndLine(pclPlaneA, v3Translate, convert(pclCentroidA));
					v3InterB = CalculateIntersectionOfPlaneAndLine(pclPlaneB, v3Translate, convert(pclCentroidA));

					m4Transform = glm::translate(m4Transform, v3InterA - v3InterB);

					break;
				}
			}

			//Apply Result
			inputB.AdjustTransformMatrix(m4Transform);

			mrResult = MERGE_OK;
		}
		else
			mrResult = MERGE_OK_NO_OPERATION;

		return mrResult;
	}

	PlaneTree::~PlaneTree()
	{
		size_t stCloudCount;

		stCloudCount = vTreeData.size();

		for (size_t i = 0; i < stCloudCount; i++)
			delete vTreeData.at(i);
	}
	
	void PlaneTree::AddPointCloud(PointCloudVector &vCloudData)
	{
		size_t stCloudCount;

		stCloudCount = vCloudData.size();

		vTreeData.resize(stCloudCount);

		for (size_t i = 0; i < stCloudCount; i++)
			vTreeData.at(i) = new std::vector<int>();

		for (size_t i = 0; i < stCloudCount; i++)
			for (size_t j = i + 1; j < stCloudCount; j++)
				if (IsIntersect(vCloudData.at(i)->GetPointCloudOBB(), vCloudData.at(j)->GetPointCloudOBB()))
					vTreeData.at(i)->push_back(j);

		pvCloudData = &vCloudData;
	}

	void PlaneTree::DoErrorCorrection(LogWindow *pLogWindow)
	{
		size_t stCloudCount;
		Stopwatch stopwatch;

		stCloudCount = vTreeData.size();

		for (size_t i = 0; i < stCloudCount; i++)
		{
			size_t stJCount;

			stJCount = vTreeData.at(i)->size();

			pLogWindow->PrintLog(L" [%d] has %d intersected data\r\n", i, stJCount);

			for (size_t j = 0; j < stJCount; j++)
			{
				glm::mat4 m4Temp;

				stopwatch.tic();
				CalculateCorrection(*pvCloudData->at(i), *pvCloudData->at(vTreeData.at(i)->at(j)));
				stopwatch.tok();

				pLogWindow->PrintLog(L"  [%d]-[%d] Correction End (%.3lfms)\r\n", i, vTreeData.at(i)->at(j), stopwatch.get());

				m4Temp = pvCloudData->at(vTreeData.at(i)->at(j))->GetAdjustMatrix();

			//	for (size_t k = vTreeData.at(i)->at(j); k < stCloudCount; k++)
			//		pvCloudData->at(k)->AdjustTransformMatrix(m4Temp);
			}

			pLogWindow->PrintLog(L"\r\n");
		}
	}
}