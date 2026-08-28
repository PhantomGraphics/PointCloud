#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Detects a plane from a point cloud using the RANSAC algorithm.
		class RansacPlaneDetector
		{
		public:
			RansacPlaneDetector() = default;
           explicit RansacPlaneDetector(unsigned seed) : hasSeed(true), rngSeed(seed) {}
			~RansacPlaneDetector() = default;

			/// @brief Holds the result of a plane detection.
			struct PlaneModel {
				Phantom::Math::Vector3df normal;  ///< Unit normal vector of the plane.
				float offset = 0.0f;              ///< Constant term in the plane equation: n*x + offset = 0.
				std::vector<size_t> inliers;      ///< Indices of inlier points.
			};

			/// @brief Detects a plane from the given point cloud using RANSAC.
			/// @param points Input point cloud.
			/// @param model Output plane model (populated on success).
			/// @param iterations Number of RANSAC iterations.
			/// @param distThreshold Distance threshold for inlier classification.
			/// @param minInliers Minimum number of inliers required for a valid model.
			/// @return true if a plane was successfully detected, false otherwise.
			bool detect(
				const std::vector<Phantom::Math::Vector3df>& points,
				PlaneModel& model,
				int iterations = 200,
				float distThreshold = 0.02f,
				size_t minInliers = 50);

		private:
           bool hasSeed = false;
			unsigned rngSeed = 0;

			/// @brief Computes the centroid of a subset of points.
			/// @param pts The full point cloud.
			/// @param indices Indices of the points to average.
			/// @return The centroid position.
			Phantom::Math::Vector3df computeCentroid(const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices) const;

			/// @brief Computes a plane normal and offset from three points.
			/// @param p0 First point.
			/// @param p1 Second point.
			/// @param p2 Third point.
			/// @param normal Output unit normal vector.
			/// @param offset Output constant term.
			/// @return true on success; false if the three points are collinear.
			bool computePlaneFromThreePoints(const Phantom::Math::Vector3df& p0, const Phantom::Math::Vector3df& p1, const Phantom::Math::Vector3df& p2, Phantom::Math::Vector3df& normal, float& offset) const;

			/// @brief Computes the distance from a point to a plane.
			/// @param p The query point.
			/// @param normal The plane normal vector.
			/// @param offset The plane constant term.
			/// @return The absolute distance from the point to the plane.
			float pointPlaneDistance(const Phantom::Math::Vector3df& p, const Phantom::Math::Vector3df& normal, float offset) const;
		};
	}
}
