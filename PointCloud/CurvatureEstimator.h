#pragma once

#include "CGLib/Math/Vector3d.h"
#include <cstddef>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Estimates the local curvature of a point cloud.
		/// Performs PCA on the neighborhood of each point and computes curvature
		/// from the ratio of eigenvalues in the normal direction.
		class CurvatureEstimator
		{
		public:
			CurvatureEstimator() = default;

			~CurvatureEstimator() = default;

			/// @brief Adds a point to be processed.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Estimates the curvature for all added points.
			/// @param searchRadius The neighborhood search radius.
			void estimate(const double searchRadius);

			/// @brief Estimates the curvature for all added points using each point's k nearest
			/// neighbors (found via Space::KDTree) instead of a fixed search radius.
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			void estimateKNN(const size_t k = 10);

			/// @brief Returns the estimated curvature values.
			/// @return A vector of curvature values in the same order as the added points.
			std::vector<double> getCurvatures() const { return curvatures; }

			/// @brief Principal curvatures (k1 >= k2) and their tangent-plane directions at a point,
			/// from fitting a local quadric (Monge patch) to its neighborhood.
			struct PrincipalCurvature
			{
				double k1 = 0.0;                              ///< Larger principal curvature.
				double k2 = 0.0;                              ///< Smaller principal curvature.
				Math::Vector3df direction1{ 0.0f, 0.0f, 0.0f }; ///< Tangent direction of k1 (zero if undetermined).
				Math::Vector3df direction2{ 0.0f, 0.0f, 0.0f }; ///< Tangent direction of k2 (zero if undetermined).
			};

			/// @brief Estimates principal curvatures (k1/k2) and their directions for all added
			/// points. For each point: builds a PCA-based normal + tangent frame from its
			/// neighborhood, fits a quadric height field z = a*u^2+b*u*v+c*v^2+d*u+e*v in that
			/// frame via least squares, then extracts k1/k2 as the eigenvalues of the resulting
			/// Hessian [[2a,b],[b,2c]] (exact for a quadric, a local approximation otherwise).
			/// Points with fewer than 6 neighbors (needed to fit the 5 quadric coefficients) get
			/// a zero-initialized PrincipalCurvature.
			/// @param searchRadius The neighborhood search radius.
			void estimatePrincipal(const double searchRadius);

			/// @brief Same as estimatePrincipal(), but finds each point's neighborhood via its k
			/// nearest neighbors (Space::KDTree) instead of a fixed search radius.
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			void estimatePrincipalKNN(const size_t k = 10);

			/// @brief Returns the estimated principal curvatures.
			/// @return A vector in the same order as the added points.
			std::vector<PrincipalCurvature> getPrincipalCurvatures() const { return principalCurvatures; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<double> curvatures;
			std::vector<PrincipalCurvature> principalCurvatures;
		};

	}
}
