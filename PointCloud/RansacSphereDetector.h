#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Detects a sphere from a point cloud using the RANSAC algorithm.
		class RansacSphereDetector
		{
		public:
			RansacSphereDetector() = default;
			explicit RansacSphereDetector(unsigned seed) : hasSeed(true), rngSeed(seed) {}
			~RansacSphereDetector() = default;

			/// @brief Holds the result of a sphere detection.
			struct SphereModel {
				Phantom::Math::Vector3df center; ///< Sphere center.
				float radius = 0.0f;             ///< Sphere radius.
				std::vector<size_t> inliers;     ///< Indices of inlier points.
			};

			/// @brief Detects a sphere from the given point cloud using RANSAC.
			/// @param points Input point cloud.
			/// @param model Output sphere model (populated on success).
			/// @param iterations Number of RANSAC iterations.
			/// @param distThreshold Radial distance threshold for inlier classification.
			/// @param minInliers Minimum number of inliers required for a valid model.
			/// @return true if a sphere was successfully detected, false otherwise.
			bool detect(
				const std::vector<Phantom::Math::Vector3df>& points,
				SphereModel& model,
				int iterations = 200,
				float distThreshold = 0.02f,
				size_t minInliers = 50);

		private:
			bool hasSeed = false;
			unsigned rngSeed = 0;

			/// @brief Computes the unique sphere passing through 4 (non-coplanar) points, via the
			/// linear system 2*(p_k-p0)*c = |p_k|^2-|p0|^2 for k=1,2,3 (center c is linear in this
			/// form; radius follows from |p0-c|).
			/// @return false if the 4 points are (near-)coplanar, making the system singular.
			bool computeSphereFromFourPoints(
				const Phantom::Math::Vector3df& p0, const Phantom::Math::Vector3df& p1,
				const Phantom::Math::Vector3df& p2, const Phantom::Math::Vector3df& p3,
				Phantom::Math::Vector3df& center, float& radius) const;
		};
	}
}
