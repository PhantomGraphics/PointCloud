#pragma once

#include "CGLib/Math/Vector3d.h"
#include <array>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Estimates Fast Point Feature Histogram (FPFH) descriptors (Rusu et al. 2009):
		/// a 33-dimensional per-point feature built from the pairwise angular relationship (a
		/// Darboux-frame-based alpha/phi/theta triple, each binned into an 11-bin histogram)
		/// between a point's normal and each of its k nearest neighbors' normals, weight-averaged
		/// with those neighbors' own histograms (Simplified PFH, SPFH) for extra robustness.
		/// Intended as the feature basis for correspondence-based global registration
		/// (RANSAC/FGR); requires per-point normals as input (e.g. from NormalEstimator).
		class FPFHEstimator
		{
		public:
			static constexpr int BinsPerFeature = 11;
			static constexpr int HistogramSize = BinsPerFeature * 3; // alpha, phi, theta

			using Histogram = std::array<float, HistogramSize>;

			FPFHEstimator() = default;
			~FPFHEstimator() = default;

			/// @brief Adds a point with its precomputed unit normal.
			void add(const Math::Vector3df& position, const Math::Vector3df& normal) {
				positions.push_back(position);
				normals.push_back(normal);
			}

			/// @brief Estimates the FPFH histogram for every added point using its k nearest
			/// neighbors (found via Space::KDTree).
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			/// @return false if there are fewer than 2 points or k < 1.
			bool estimate(size_t k);

			/// @brief Returns the estimated histograms, same order as added points.
			std::vector<Histogram> getHistograms() const { return histograms; }

		private:
			std::vector<Math::Vector3df> positions;
			std::vector<Math::Vector3df> normals;
			std::vector<Histogram> histograms;

			/// @brief Computes the 3 pairwise angular features (Darboux frame) between a "source"
			/// point/normal and a "target" point/normal, following Rusu et al.'s convention with
			/// the source point's own normal always used as the frame's u-axis. This is a
			/// simplification of PCL's canonical source/target swap (which picks whichever of the
			/// two normals is more aligned with the connecting vector as the frame origin); it is
			/// adequate here because this class always queries pairs consistently from the same
			/// point's perspective, but the result is not guaranteed symmetric under argument swap.
			/// @return false if the two positions coincide or the connecting vector is (nearly)
			///         parallel to sourceNormal (a degenerate Darboux frame).
			static bool computePairFeatures(
				const Math::Vector3df& sourcePosition, const Math::Vector3df& sourceNormal,
				const Math::Vector3df& targetPosition, const Math::Vector3df& targetNormal,
				float& outAlpha, float& outPhi, float& outTheta);

			/// @brief Computes the Simplified Point Feature Histogram (SPFH) for point `index`
			/// against its listed neighbor indices.
			static Histogram computeSPFH(
				size_t index,
				const std::vector<int>& neighborIndices,
				const std::vector<Math::Vector3df>& positions,
				const std::vector<Math::Vector3df>& normals);
		};

	}
}
