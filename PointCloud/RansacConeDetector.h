#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Detects a cone from a point cloud using the RANSAC algorithm.
		class RansacConeDetector
		{
		public:
			RansacConeDetector() = default;
			explicit RansacConeDetector(unsigned seed) : hasSeed(true), rngSeed(seed) {}
			~RansacConeDetector() = default;

			/// @brief Holds the result of a cone detection.
			struct ConeModel {
				Phantom::Math::Vector3df apex;      ///< Cone apex.
				Phantom::Math::Vector3df axis;      ///< Unit axis direction, pointing from the apex toward the base (increasing radius).
				float halfAngleRad = 0.0f;          ///< Half opening angle, in radians.
				std::vector<size_t> inliers;        ///< Indices of inlier points.
			};

			/// @brief Detects a cone from the given point cloud using RANSAC.
			/// @param points Input point cloud.
			/// @param model Output cone model (populated on success).
			/// @param iterations Number of RANSAC iterations.
			/// @param distThreshold Radial distance threshold (from the cone surface) for inlier classification.
			/// @param minInliers Minimum number of inliers required for a valid model.
			/// @return true if a cone was successfully detected, false otherwise.
			bool detect(
				const std::vector<Phantom::Math::Vector3df>& points,
				ConeModel& model,
				int iterations = 200,
				float distThreshold = 0.02f,
				size_t minInliers = 50);

		private:
			bool hasSeed = false;
			unsigned rngSeed = 0;

			/// @brief Computes the centroid of a subset of points.
			Phantom::Math::Vector3df computeCentroid(const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices) const;

			/// @brief Estimates the dominant direction (largest-eigenvalue eigenvector of the
			/// covariance matrix) of a point subset via power iteration. For a cone sample
			/// elongated along its axis, this approximates the axis direction (sign ambiguous).
			Phantom::Math::Vector3df estimateDominantDirection(const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices) const;

			/// @brief Fits a cone (apex, oriented axis, half-angle) to a point subset, given an
			/// (sign-ambiguous) axis direction estimate. Exploits that a cone's radius grows
			/// linearly with distance along the axis from the apex: fits a line
			/// radius = m*axialDistance + b via least squares over the subset, then recovers the
			/// apex (radius == 0 crossing), the correctly-signed axis (increasing-radius
			/// direction), and the half-angle (atan(|m|)).
			/// @return false if the subset's axial spread is degenerate (near-zero slope denominator).
			bool fitConeFromDirection(
				const std::vector<Phantom::Math::Vector3df>& pts, const std::vector<size_t>& indices,
				const Phantom::Math::Vector3df& roughDirection,
				Phantom::Math::Vector3df& apex, Phantom::Math::Vector3df& axis, float& halfAngleRad) const;

			/// @brief Computes the perpendicular distance from a point to a line.
			float pointLineDistance(const Phantom::Math::Vector3df& p, const Phantom::Math::Vector3df& origin, const Phantom::Math::Vector3df& dir) const;
		};
	}
}
