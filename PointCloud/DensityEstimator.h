#pragma once

#include <cstddef>
#include <vector>
#include "CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Estimates the local density of a point cloud.
		/// For each point, counts the number of neighbors within a given search radius
		/// and uses that count as the density value.
		class DensityEstimator
		{
		public:
			DensityEstimator() = default;

			~DensityEstimator() = default;

			/// @brief Adds a point to the estimation target.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Estimates the density for all added points.
			/// @param searchRadius The neighborhood search radius.
			void estimate(const double searchRadius);

			/// @brief Estimates the density for all added points using each point's k nearest
			/// neighbors (found via Space::KDTree) instead of a fixed search radius. The
			/// Gaussian weighting kernel is normalized by the distance to the farthest of the k
			/// neighbors (the KNN equivalent of the fixed searchRadius used by estimate()).
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			void estimateKNN(const size_t k = 10);

			/// @brief Returns the estimated density values.
			/// @return A vector of density values in the same order as the added points.
			std::vector<double> getDensities() const { return densities; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<double> densities;
		};
	}
}
