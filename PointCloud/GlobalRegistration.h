#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Matrix3d.h"
#include "FPFHEstimator.h"

namespace Phantom {
	namespace PC {

		/// @brief Coarse rigid alignment of a source point cloud onto a target point cloud from
		/// FPFH feature correspondences via RANSAC (Rusu et al.'s FPFH+RANSAC, the same approach
		/// as Open3D's "global registration" / PCL's SampleConsensusInitialAlignment), requiring
		/// no initial pose guess. Meant as the initialization step feeding
		/// ICPRegistration::align()/alignPointToPlane() for fine local refinement, since ICP by
		/// itself needs a reasonably close starting pose to converge to the correct optimum.
		class GlobalRegistration
		{
		public:
			GlobalRegistration() = default;
			explicit GlobalRegistration(unsigned seed) : hasSeed(true), rngSeed(seed) {}
			~GlobalRegistration() = default;

			/// @brief Holds the result of a global registration attempt.
			struct Result {
				Phantom::Math::Matrix3df rotation = Phantom::Math::identitiyMatrix3d<float>(); ///< Rigid rotation.
				Phantom::Math::Vector3df translation{ 0.0f, 0.0f, 0.0f };                       ///< Rigid translation.
				size_t inlierCount = 0;  ///< Number of feature correspondences consistent with the final transform.
				float inlierRmse = 0.0f; ///< RMS distance of those correspondences under the final transform.
			};

			/// @brief Estimates a coarse rigid transform aligning `source` onto `target`, using
			/// one-way nearest-neighbor correspondences in FPFH feature space followed by RANSAC
			/// over that correspondence set (geometric consistency, not feature distance, decides
			/// the winner: feature nearest-neighbor matches are noisy, so most sampled
			/// correspondence subsets are wrong and are expected to be rejected as they fail to
			/// explain the rest of the correspondence set under the candidate rigid transform).
			/// @param source Source point positions.
			/// @param sourceFeatures Per-point FPFH histograms for `source` (same size/order; see FPFHEstimator).
			/// @param target Target point positions.
			/// @param targetFeatures Per-point FPFH histograms for `target` (same size/order).
			/// @param result Output. Always populated when this returns true.
			/// @param iterations Number of RANSAC iterations.
			/// @param maxCorrespondenceDistance Max positional distance (after applying a candidate
			///        transform) for a feature correspondence to count as an inlier.
			/// @param sampleSize Number of correspondences sampled per RANSAC iteration (>= 3; a
			///        rigid transform needs at least 3 non-collinear correspondences to be determined).
			/// @param edgeLengthTolerance Cheap correspondence-set pruning applied before fitting
			///        (Open3D's "correspondence checker" pattern): a sampled correspondence set is
			///        rejected if any pair's source/target edge lengths differ by more than this
			///        fractional tolerance, so obviously-inconsistent feature matches are skipped
			///        before the O(N) inlier count.
			/// @param minInliers Minimum inlier correspondences required to accept a model.
			/// @return true if a transform with at least `minInliers` correspondences was found.
			bool align(
				const std::vector<Phantom::Math::Vector3df>& source,
				const std::vector<FPFHEstimator::Histogram>& sourceFeatures,
				const std::vector<Phantom::Math::Vector3df>& target,
				const std::vector<FPFHEstimator::Histogram>& targetFeatures,
				Result& result,
				int iterations = 1000,
				float maxCorrespondenceDistance = 0.05f,
				size_t sampleSize = 3,
				float edgeLengthTolerance = 0.15f,
				size_t minInliers = 3) const;

		private:
			bool hasSeed = false;
			unsigned rngSeed = 0;

			/// @brief Squared L2 distance between two FPFH histograms.
			static float featureDistanceSq(const FPFHEstimator::Histogram& a, const FPFHEstimator::Histogram& b);

			/// @brief For each source point, finds the index of its nearest neighbor in
			/// `targetFeatures` by brute-force L2 distance over the 33-dim FPFH space (no spatial
			/// structure applies to descriptor space the way Space::KDTree does to 3D
			/// positions).
			static std::vector<size_t> matchFeatures(
				const std::vector<FPFHEstimator::Histogram>& sourceFeatures,
				const std::vector<FPFHEstimator::Histogram>& targetFeatures);
		};

	}
}
