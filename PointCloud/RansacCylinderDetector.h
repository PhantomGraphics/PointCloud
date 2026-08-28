#pragma once

#include <vector>
#include <random>
#include <cstddef>
#include "../../CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Detects a cylinder from a point cloud using the RANSAC algorithm.
		class RansacCylinderDetector
		{
		public:
			RansacCylinderDetector() = default;
			~RansacCylinderDetector() = default;

			/// @brief Holds the result of a cylinder detection.
			struct CylinderModel {
				Phantom::Math::Vector3df origin;    ///< Base point of the cylinder axis (centroid of the point cloud).
				Phantom::Math::Vector3df direction; ///< Unit direction vector of the cylinder axis.
				float radius = 0.0f;               ///< Estimated cylinder radius.
				std::vector<size_t> inliers;       ///< Indices of inlier points.
			};

			/// @brief Detects a cylinder from the given point cloud using RANSAC.
			/// @param points Input point cloud.
			/// @param model Output cylinder model (populated on success).
			/// @param iterations Number of RANSAC iterations.
			/// @param distThreshold Radial distance threshold for inlier classification.
			/// @param minInliers Minimum number of inliers required for a valid model.
			/// @return true if a cylinder was successfully detected, false otherwise.
			bool detect(
				const std::vector<Phantom::Math::Vector3df>& points,
				CylinderModel& model,
				int iterations = 200,
				float distThreshold = 0.02f,
				size_t minInliers = 50);

		private:
			/// @brief Computes the centroid of a subset of points.
			/// @param pts The full point cloud.
			/// @param indices Indices of the points to average.
			/// @return The centroid position.
			Phantom::Math::Vector3df computeCentroid(const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices) const;

			/// @brief Estimates the principal direction of a point subset using power iteration.
			/// @param pts The full point cloud.
			/// @param indices Indices of the points to analyze.
			/// @return The unit vector corresponding to the largest eigenvalue.
			Phantom::Math::Vector3df estimatePrincipalDirection(const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices) const;

			/// @brief Computes the distance from a point to a line.
			/// @param p The query point.
			/// @param origin A point on the line.
			/// @param dir The unit direction vector of the line.
			/// @return The shortest distance from the point to the line.
			float pointLineDistance(const Phantom::Math::Vector3df& p, const Phantom::Math::Vector3df& origin, const Phantom::Math::Vector3df& dir) const;
		};
	}
}
