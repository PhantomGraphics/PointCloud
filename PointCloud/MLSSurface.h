#pragma once

#include "CGLib/Math/Vector3d.h"
#include <cstddef>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Moving Least Squares (MLS) surface fitting for smoothing and upsampling a
		/// point cloud. At each point, a local height field over a PCA-derived tangent plane is
		/// fit in a weighted least-squares sense (Gaussian weighting by neighbor distance) --
		/// a quadric (6 coefficients) when enough neighbors are available, otherwise a plane (3
		/// coefficients). smooth() projects each original point onto its own fitted surface;
		/// upsample() additionally samples new points across that surface on a regular grid.
		class MLSSurface
		{
		public:
			MLSSurface() = default;
			~MLSSurface() = default;

			/// @brief Adds a point to be processed.
			void add(const Math::Vector3df& position) { pointCloud.push_back(position); }

			/// @brief Smooths every added point by projecting it onto a locally-fit weighted
			/// height field. Points with fewer than 2 actual neighbors (can't build a PCA tangent
			/// frame) are left at their original position. The Gaussian weighting kernel's
			/// standard deviation is fixed at searchRadius/2.
			/// @param searchRadius The neighborhood search radius.
			void smooth(const double searchRadius);

			/// @brief Same as smooth(), but finds each point's neighborhood via its k nearest
			/// neighbors (Space::KDTree) instead of a fixed search radius. The Gaussian
			/// weighting kernel's standard deviation is fixed at (distance to the farthest of the
			/// k neighbors)/2 -- the KNN equivalent of the fixed searchRadius/2 used by smooth().
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			void smoothKNN(const size_t k = 10);

			/// @brief Returns the smoothed points, same order/count as added points. Only
			/// meaningful after smooth() has been called; points that were left unchanged (too
			/// few neighbors) keep their original coordinates.
			std::vector<Math::Vector3df> getSmoothedPoints() const { return smoothedPoints; }

			/// @brief Generates additional points by resampling the locally-fit surface (see
			/// smooth()) around each added point on a regular grid within the tangent plane, out
			/// to upsampleRadius and spaced stepSize apart. Independent of smooth(): rebuilds its
			/// own neighborhoods and does not require smooth() to have been called first.
			/// @param searchRadius The neighborhood search radius used to fit each local surface.
			/// @param upsampleRadius Tangent-plane radius within which new samples are generated
			///        around each original point.
			/// @param stepSize Spacing between generated samples; must be > 0.
			/// @return The newly generated points only (original points are not included).
			std::vector<Math::Vector3df> upsample(const double searchRadius, const float upsampleRadius, const float stepSize) const;

			/// @brief Same as upsample(), but finds each point's neighborhood via its k nearest
			/// neighbors (Space::KDTree) instead of a fixed search radius.
			/// @param k Number of nearest neighbors (excluding the point itself) used to fit each
			///          local surface.
			/// @param upsampleRadius Tangent-plane radius within which new samples are generated
			///        around each original point.
			/// @param stepSize Spacing between generated samples; must be > 0.
			/// @return The newly generated points only (original points are not included).
			std::vector<Math::Vector3df> upsampleKNN(const size_t k, const float upsampleRadius, const float stepSize) const;

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<Math::Vector3df> smoothedPoints;
		};

	}
}
