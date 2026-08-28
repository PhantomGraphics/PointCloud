#pragma once

#include "CGLib/Math/Vector3d.h"
#include <cstddef>
#include <vector>

namespace Phantom {
	namespace PC {

		class NormalEstimator
		{
		public:
			NormalEstimator() = default;

			~NormalEstimator() = default;

			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			void estimate(const double searchRadius);

			/// @brief Estimates the normal for all added points using each point's k nearest
			/// neighbors (found via Space::KDTree) instead of a fixed search radius.
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			void estimateKNN(const size_t k = 10);

			/// @brief Flips each already-estimated normal (see estimate()) so it points toward
			/// `viewpoint`, resolving the sign ambiguity inherent to PCA-based normal estimation
			/// (the covariance eigenvector is only defined up to sign). Points whose normal could
			/// not be estimated (zero vector, e.g. too few neighbors) are left unchanged.
			/// @param viewpoint The reference position (e.g. sensor/scanner origin) every normal
			///        should face toward.
			void orientTowardsViewpoint(const Math::Vector3df& viewpoint);

			std::vector<Math::Vector3df> getNormals() const { return normals; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<Math::Vector3df> normals;
		};

	}
}