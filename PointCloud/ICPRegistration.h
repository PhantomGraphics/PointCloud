#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Matrix3d.h"
#include "../../CGLib/Space/Space/KDTree.h"
#include "ProgressReporter.h"

namespace Phantom {
	namespace PC {

		/// @brief Aligns a source point cloud onto a target point cloud using Iterative
		/// Closest Point (ICP), either the point-to-point (align()) or point-to-plane
		/// (alignPointToPlane()) variant, both rigid by default with optional uniform-scale
		/// estimation (align() only) and optional robust-kernel down-weighting of
		/// correspondences.
		class ICPRegistration
		{
		public:
			ICPRegistration() = default;
			~ICPRegistration() = default;

			/// @brief M-estimator applied to correspondence residuals before fitting the
			/// transform for that iteration, on top of the hard `maxCorrespondenceDistance`
			/// rejection. None disables it (every accepted correspondence gets weight 1).
			enum class RobustKernel {
				None,
				Huber, ///< weight = 1 for |residual| <= delta, else delta / |residual|.
				Tukey  ///< weight = (1 - (residual/delta)^2)^2 for |residual| < delta, else 0.
			};

			/// @brief Holds the result of an ICP alignment.
			struct Result {
				Phantom::Math::Matrix3df rotation = Phantom::Math::identitiyMatrix3d<float>(); ///< Accumulated rotation.
				Phantom::Math::Vector3df translation{ 0.0f, 0.0f, 0.0f };                       ///< Accumulated translation.
				float scale = 1.0f;       ///< Accumulated uniform scale (Umeyama). Always 1.0 unless `estimateScale` is enabled.
				int iterations = 0;      ///< Number of iterations actually performed.
				float fitness = 0.0f;    ///< Mean squared nearest-neighbor distance at the final iteration.
				bool converged = false;  ///< true if the fitness improvement fell below `tolerance` before `maxIterations`.
			};

			/// @brief Aligns `source` onto `target` minimizing point-to-point distance.
			/// @param source Points to be registered; not modified in place (see Result for the fitted transform).
			/// @param target Fixed reference point cloud.
			/// @param result Output alignment result. Always populated when this returns true.
			/// @param maxIterations Maximum number of ICP iterations.
			/// @param tolerance Stops early once the change in fitness between iterations drops below this.
			/// @param maxCorrespondenceDistance Correspondences farther apart than this are rejected as outliers;
			///        a value <= 0 disables rejection (every point is matched to its nearest neighbor).
			/// @param robustKernel Down-weights accepted correspondences by residual distance (see RobustKernel).
			/// @param robustKernelDelta Scale parameter (in distance units) for `robustKernel`; ignored when None.
			/// @param estimateScale When true, also fits a uniform scale (Umeyama) instead of assuming rigid.
			/// @param reporter Optional progress/cancellation callback (F-3), reported once per
			///                 completed outer iteration (maxIterations is typically small, ~50,
			///                 so no throttling is needed unlike the per-point clustering loops).
			///                 If cancelled partway through, `result` still holds the transform
			///                 accumulated up to the last completed iteration (same as a normal
			///                 early exit via convergence).
			/// @return true if at least one iteration found enough correspondences (>= 3) to fit a rotation.
			bool align(
				const std::vector<Phantom::Math::Vector3df>& source,
				const std::vector<Phantom::Math::Vector3df>& target,
				Result& result,
				int maxIterations = 50,
				float tolerance = 1.0e-6f,
				float maxCorrespondenceDistance = 0.0f,
				RobustKernel robustKernel = RobustKernel::None,
				float robustKernelDelta = 1.0f,
				bool estimateScale = false,
				const ProgressReporter& reporter = {}) const;

			/// @brief Aligns `source` onto `target` minimizing point-to-plane distance (residual
			/// projected onto the target surface normal at each correspondence), using a
			/// Gauss-Newton linearization solved once per iteration. Converges faster and more
			/// robustly than point-to-point when target normals are available and reasonably
			/// accurate. Rigid only (no scale estimation).
			/// @param source Points to be registered; not modified in place.
			/// @param target Fixed reference point cloud.
			/// @param targetNormals Per-point normals of `target` (same size as `target`, unit length expected).
			/// @param result Output alignment result. Always populated when this returns true.
			/// @param maxIterations Maximum number of ICP iterations.
			/// @param tolerance Stops early once the change in fitness between iterations drops below this.
			/// @param maxCorrespondenceDistance Correspondences farther apart than this are rejected as outliers;
			///        a value <= 0 disables rejection.
			/// @param robustKernel Down-weights accepted correspondences by point-to-plane residual (see RobustKernel).
			/// @param robustKernelDelta Scale parameter (in distance units) for `robustKernel`; ignored when None.
			/// @return true if at least one iteration found enough correspondences (>= 6, one per
			///         linear-system unknown) and the resulting 6x6 normal-equation system was solvable.
			bool alignPointToPlane(
				const std::vector<Phantom::Math::Vector3df>& source,
				const std::vector<Phantom::Math::Vector3df>& target,
				const std::vector<Phantom::Math::Vector3df>& targetNormals,
				Result& result,
				int maxIterations = 50,
				float tolerance = 1.0e-6f,
				float maxCorrespondenceDistance = 0.0f,
				RobustKernel robustKernel = RobustKernel::None,
				float robustKernelDelta = 1.0f) const;

			/// @brief Applies a Result's transform (rotation, scale, translation) to a single point.
			static Phantom::Math::Vector3df transformPoint(const Result& result, const Phantom::Math::Vector3df& p);

			/// @brief Fits the weighted similarity transform minimizing
			/// sum w[i] * |scale*R*src[i]+t - dst[i]|^2 (weighted Kabsch/Umeyama). `weights` must
			/// be the same size as `src`/`dst`, all non-negative, and not sum to (near) zero.
			/// When `estimateScale` is false, `outScale` is always 1.0. Public (rather than an
			/// align()-only implementation detail) since it's also the minimal-sample rigid fit
			/// used by GlobalRegistration's RANSAC loop.
			static bool computeRigidTransform(
				const std::vector<Phantom::Math::Vector3df>& src,
				const std::vector<Phantom::Math::Vector3df>& dst,
				const std::vector<float>& weights,
				bool estimateScale,
				Phantom::Math::Matrix3df& outRotation,
				Phantom::Math::Vector3df& outTranslation,
				float& outScale);

		private:
			/// @brief Nearest neighbor search against `target`, accelerated by `targetTree`
			/// (a Phantom::Space::KDTree already built over `target`, one per align() call rather
			/// than per query).
			static bool findClosestPoint(
				const Phantom::Math::Vector3df& query,
				const std::vector<Phantom::Math::Vector3df>& target,
				const Phantom::Space::KDTree& targetTree,
				size_t& outIndex,
				float& outSqDist);

			/// @brief Evaluates the M-estimator weight for a residual magnitude (see RobustKernel).
			static double robustWeight(RobustKernel kernel, float delta, float residual);

			/// @brief Solves the 6x6 system A*x = b via Gauss-Jordan elimination with partial
			/// pivoting. `A` and `b` are consumed (overwritten as scratch space).
			/// @return false if `A` is (numerically) singular.
			static bool solveNormalEquations6(double A[6][6], double b[6], double outX[6]);

			/// @brief Converts a rotation vector (axis * angle, radians) to a rotation matrix
			/// via Rodrigues' formula. Returns identity for a near-zero vector.
			static Phantom::Math::Matrix3dd axisAngleToRotation(const Phantom::Math::Vector3dd& r);
		};
	}
}
