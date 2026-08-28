#include "ICPRegistration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <CGLib/Numerics/Numerics/SVD3d.h>

using namespace Phantom::PC;
using namespace Phantom::Math;
using namespace Phantom::Space;

bool ICPRegistration::align(
	const std::vector<Vector3df>& source,
	const std::vector<Vector3df>& target,
	Result& result,
	int maxIterations,
	float tolerance,
	float maxCorrespondenceDistance,
	RobustKernel robustKernel,
	float robustKernelDelta,
	bool estimateScale,
	const ProgressReporter& reporter) const
{
	result = Result{};
	if (source.empty() || target.empty() || maxIterations <= 0) {
		return false;
	}

	const bool rejectOutliers = maxCorrespondenceDistance > 0.0f;
	const float maxDistSq = maxCorrespondenceDistance * maxCorrespondenceDistance;

	// `target` is fixed across all iterations, so the KD-tree is built once here rather than
	// per query (O(N log M) per iteration instead of the previous O(N*M) brute force).
	KDTree targetTree;
	targetTree.build(target);

	std::vector<Vector3df> current = source;
	Matrix3df totalRotation = identitiyMatrix3d<float>();
	Vector3df totalTranslation(0.0f, 0.0f, 0.0f);
	float totalScale = 1.0f;

	float lastFitness = 0.0f;
	bool hasFitness = false;
	bool anyCorrespondence = false;
	int completedIterations = 0;

	for (int iter = 0; iter < maxIterations; ++iter) {
		std::vector<Vector3df> matchedSrc;
		std::vector<Vector3df> matchedDst;
		std::vector<float> matchedWeight;
		matchedSrc.reserve(current.size());
		matchedDst.reserve(current.size());
		matchedWeight.reserve(current.size());
		double sumSqDist = 0.0;

		// Correspondence search is independent per point (targetTree is built once above and only
		// queried, never mutated), but the accept/reject decision and matched-point data can't be
		// push_back'ed directly from a parallel loop. So each point's verdict is computed into
		// per-index scratch arrays first, then folded into matchedSrc/matchedDst/matchedWeight and
		// summed into sumSqDist single-threaded below, in the original point order -- same pattern
		// as SORFilter/RadiusOutlierFilter's classification loops, and it keeps the accumulation
		// bit-identical to the pre-parallelization serial result (not just tolerance-close).
		const int n = static_cast<int>(current.size());
		std::vector<char> accepted(static_cast<size_t>(n));
		std::vector<size_t> matchIdx(static_cast<size_t>(n));
		std::vector<float> matchSqDist(static_cast<size_t>(n));

#pragma omp parallel for
		for (int i = 0; i < n; ++i) {
			size_t idx = 0;
			float sqDist = 0.0f;
			bool ok = findClosestPoint(current[static_cast<size_t>(i)], target, targetTree, idx, sqDist);
			if (ok && rejectOutliers && sqDist > maxDistSq) ok = false;
			accepted[static_cast<size_t>(i)] = ok ? 1 : 0;
			if (ok) {
				matchIdx[static_cast<size_t>(i)] = idx;
				matchSqDist[static_cast<size_t>(i)] = sqDist;
			}
		}

		for (int i = 0; i < n; ++i) {
			if (!accepted[static_cast<size_t>(i)]) continue;
			const size_t idx = matchIdx[static_cast<size_t>(i)];
			const float sqDist = matchSqDist[static_cast<size_t>(i)];
			matchedSrc.push_back(current[static_cast<size_t>(i)]);
			matchedDst.push_back(target[idx]);
			matchedWeight.push_back(static_cast<float>(robustWeight(robustKernel, robustKernelDelta, std::sqrt(sqDist))));
			sumSqDist += static_cast<double>(sqDist);
		}

		if (matchedSrc.size() < 3) {
			break; // Not enough correspondences left to fit a rotation.
		}
		anyCorrespondence = true;
		completedIterations = iter + 1;

		const float fitness = static_cast<float>(sumSqDist / static_cast<double>(matchedSrc.size()));

		Matrix3df stepRotation;
		Vector3df stepTranslation;
		float stepScale = 1.0f;
		if (!computeRigidTransform(matchedSrc, matchedDst, matchedWeight, estimateScale, stepRotation, stepTranslation, stepScale)) {
			break;
		}

		for (auto& p : current) {
			p = stepScale * (stepRotation * p) + stepTranslation;
		}
		totalRotation = stepRotation * totalRotation;
		totalTranslation = stepScale * (stepRotation * totalTranslation) + stepTranslation;
		totalScale = stepScale * totalScale;

		const bool converged = hasFitness && std::fabs(lastFitness - fitness) < tolerance;
		lastFitness = fitness;
		hasFitness = true;

		if (converged) {
			result.converged = true;
			break;
		}

		// maxIterations is normally small (default 50), so unlike the per-point
		// clustering loops this reports every iteration without throttling (F-3).
		// On cancellation, `result` keeps the transform accumulated so far
		// (completedIterations worth), same as any other early exit above.
		if (!reporter.report(static_cast<float>(iter + 1) / static_cast<float>(maxIterations))) {
			break;
		}
	}

	if (!anyCorrespondence) {
		return false;
	}

	result.rotation = totalRotation;
	result.translation = totalTranslation;
	result.scale = totalScale;
	result.iterations = completedIterations;
	result.fitness = lastFitness;
	return true;
}

bool ICPRegistration::alignPointToPlane(
	const std::vector<Vector3df>& source,
	const std::vector<Vector3df>& target,
	const std::vector<Vector3df>& targetNormals,
	Result& result,
	int maxIterations,
	float tolerance,
	float maxCorrespondenceDistance,
	RobustKernel robustKernel,
	float robustKernelDelta) const
{
	result = Result{};
	if (source.empty() || target.empty() || maxIterations <= 0) {
		return false;
	}
	if (targetNormals.size() != target.size()) {
		return false;
	}

	const bool rejectOutliers = maxCorrespondenceDistance > 0.0f;
	const float maxDistSq = maxCorrespondenceDistance * maxCorrespondenceDistance;

	KDTree targetTree;
	targetTree.build(target);

	std::vector<Vector3df> current = source;
	Matrix3df totalRotation = identitiyMatrix3d<float>();
	Vector3df totalTranslation(0.0f, 0.0f, 0.0f);

	// Unlike point-to-point's Kabsch step (an exact per-iteration optimum), each
	// point-to-plane step is only a local Gauss-Newton linearization: near convergence, tiny
	// residuals make the fit sensitive to correspondence reassignment noise and it can
	// oscillate or drift instead of settling. So every cumulative transform (including the
	// starting identity and the very last applied step) is fitness-evaluated below, and the
	// best one found is what gets returned rather than unconditionally trusting the last.
	Matrix3df bestRotation = totalRotation;
	Vector3df bestTranslation = totalTranslation;
	float bestFitness = std::numeric_limits<float>::max();
	int bestIterations = 0;

	float lastFitness = 0.0f;
	bool hasFitness = false;
	bool anyCorrespondence = false;

	// Runs one extra pass beyond maxIterations: that last pass only evaluates the transform
	// resulting from the maxIterations-th applied step, without computing/applying another.
	for (int iter = 0; iter <= maxIterations; ++iter) {
		double AtA[6][6] = {};
		double Atb[6] = {};
		double sumSqDist = 0.0;
		size_t matchCount = 0;

		// Same split as align()'s correspondence loop: MSVC's OpenMP 2.0 has no array/matrix
		// reduction clause, so each point's linearized row [(s x n); n] and residual are computed
		// into scratch arrays in parallel, then folded into AtA/Atb/sumSqDist/matchCount
		// single-threaded below in the original point order (bit-identical to the serial result).
		const int n = static_cast<int>(current.size());
		std::vector<char> accepted(static_cast<size_t>(n));
		std::vector<std::array<double, 6>> matchRow(static_cast<size_t>(n));
		std::vector<double> matchResidual(static_cast<size_t>(n));
		std::vector<double> matchWeight(static_cast<size_t>(n));
		std::vector<float> matchSqDist(static_cast<size_t>(n));

#pragma omp parallel for
		for (int i = 0; i < n; ++i) {
			const auto& p = current[static_cast<size_t>(i)];
			size_t idx = 0;
			float sqDist = 0.0f;
			bool ok = findClosestPoint(p, target, targetTree, idx, sqDist);
			if (ok && rejectOutliers && sqDist > maxDistSq) ok = false;
			if (!ok) {
				accepted[static_cast<size_t>(i)] = 0;
				continue;
			}

			const Vector3dd s(p);
			const Vector3dd nrm(targetNormals[idx]);
			const Vector3dd d(target[idx]);

			// Point-to-plane residual: n . (d - s). Linearizing a small incremental rotation
			// r (axis*angle) and translation t around the current pose gives the row
			// [ (s x n) ; n ] . [r; t] = n . (d - s).
			const double residual = glm::dot(nrm, d - s);
			const double w = robustWeight(robustKernel, robustKernelDelta, static_cast<float>(residual));
			if (w <= 0.0) {
				accepted[static_cast<size_t>(i)] = 0;
				continue;
			}

			const Vector3dd sxn = glm::cross(s, nrm);
			matchRow[static_cast<size_t>(i)] = { sxn.x, sxn.y, sxn.z, nrm.x, nrm.y, nrm.z };
			matchResidual[static_cast<size_t>(i)] = residual;
			matchWeight[static_cast<size_t>(i)] = w;
			matchSqDist[static_cast<size_t>(i)] = sqDist;
			accepted[static_cast<size_t>(i)] = 1;
		}

		for (int i = 0; i < n; ++i) {
			if (!accepted[static_cast<size_t>(i)]) continue;
			const auto& a = matchRow[static_cast<size_t>(i)];
			const double w = matchWeight[static_cast<size_t>(i)];
			const double residual = matchResidual[static_cast<size_t>(i)];
			for (int row = 0; row < 6; ++row) {
				for (int col = 0; col < 6; ++col) {
					AtA[row][col] += w * a[static_cast<size_t>(row)] * a[static_cast<size_t>(col)];
				}
				Atb[row] += w * a[static_cast<size_t>(row)] * residual;
			}

			++matchCount;
			sumSqDist += static_cast<double>(matchSqDist[static_cast<size_t>(i)]);
		}

		if (matchCount < 6) {
			break; // Not enough correspondences to constrain the 6-unknown linear system.
		}
		anyCorrespondence = true;

		const float fitness = static_cast<float>(sumSqDist / static_cast<double>(matchCount));
		if (fitness < bestFitness) {
			bestFitness = fitness;
			bestRotation = totalRotation;
			bestTranslation = totalTranslation;
			bestIterations = iter;
		}

		const bool converged = hasFitness && std::fabs(lastFitness - fitness) < tolerance;
		lastFitness = fitness;
		hasFitness = true;

		if (converged) {
			result.converged = true;
			break;
		}
		if (iter == maxIterations) {
			break; // That was the final evaluation-only pass; no step left to compute.
		}

		double solution[6] = {};
		if (!solveNormalEquations6(AtA, Atb, solution)) {
			break;
		}

		const Vector3dd rvec(solution[0], solution[1], solution[2]);
		const Vector3dd tvec(solution[3], solution[4], solution[5]);
		const Matrix3df stepRotation = Matrix3df(axisAngleToRotation(rvec));
		const Vector3df stepTranslation(
			static_cast<float>(tvec.x),
			static_cast<float>(tvec.y),
			static_cast<float>(tvec.z));

		for (auto& p : current) {
			p = stepRotation * p + stepTranslation;
		}
		totalRotation = stepRotation * totalRotation;
		totalTranslation = stepRotation * totalTranslation + stepTranslation;
	}

	if (!anyCorrespondence) {
		return false;
	}

	result.rotation = bestRotation;
	result.translation = bestTranslation;
	result.scale = 1.0f;
	result.iterations = bestIterations;
	result.fitness = bestFitness;
	return true;
}

Vector3df ICPRegistration::transformPoint(const Result& result, const Vector3df& p)
{
	return result.scale * (result.rotation * p) + result.translation;
}

bool ICPRegistration::findClosestPoint(
	const Vector3df& query,
	const std::vector<Vector3df>& target,
	const KDTree& targetTree,
	size_t& outIndex,
	float& outSqDist)
{
	if (target.empty()) return false;

	const int idx = targetTree.findNearestIndex(query);
	if (idx < 0) return false;

	outIndex = static_cast<size_t>(idx);
	const auto diff = target[outIndex] - query;
	outSqDist = glm::dot(diff, diff);
	return true;
}

double ICPRegistration::robustWeight(RobustKernel kernel, float delta, float residual)
{
	if (kernel == RobustKernel::None || delta <= 0.0f) return 1.0;

	const double r = std::fabs(static_cast<double>(residual));
	const double d = static_cast<double>(delta);
	switch (kernel) {
	case RobustKernel::Huber:
		return (r <= d) ? 1.0 : d / r;
	case RobustKernel::Tukey: {
		if (r >= d) return 0.0;
		const double t = r / d;
		const double w = 1.0 - t * t;
		return w * w;
	}
	default:
		return 1.0;
	}
}

bool ICPRegistration::solveNormalEquations6(double A[6][6], double b[6], double outX[6])
{
	for (int col = 0; col < 6; ++col) {
		int pivot = col;
		double best = std::fabs(A[col][col]);
		for (int r = col + 1; r < 6; ++r) {
			const double v = std::fabs(A[r][col]);
			if (v > best) { best = v; pivot = r; }
		}
		if (best < 1.0e-12) return false;

		if (pivot != col) {
			std::swap(A[pivot], A[col]);
			std::swap(b[pivot], b[col]);
		}

		const double diag = A[col][col];
		for (int r = 0; r < 6; ++r) {
			if (r == col) continue;
			const double factor = A[r][col] / diag;
			if (factor == 0.0) continue;
			for (int c = col; c < 6; ++c) {
				A[r][c] -= factor * A[col][c];
			}
			b[r] -= factor * b[col];
		}
	}

	for (int i = 0; i < 6; ++i) {
		outX[i] = b[i] / A[i][i];
	}
	return true;
}

Matrix3dd ICPRegistration::axisAngleToRotation(const Vector3dd& r)
{
	const double angle = glm::length(r);
	if (angle < 1.0e-12) {
		return identitiyMatrix3d<double>();
	}

	const Vector3dd axis = r / angle;
	const double c = std::cos(angle);
	const double s = std::sin(angle);
	const double t = 1.0 - c;

	return Matrix3dd(
		t * axis.x * axis.x + c,        t * axis.x * axis.y - s * axis.z, t * axis.x * axis.z + s * axis.y,
		t * axis.x * axis.y + s * axis.z, t * axis.y * axis.y + c,        t * axis.y * axis.z - s * axis.x,
		t * axis.x * axis.z - s * axis.y, t * axis.y * axis.z + s * axis.x, t * axis.z * axis.z + c);
}

bool ICPRegistration::computeRigidTransform(
	const std::vector<Vector3df>& src,
	const std::vector<Vector3df>& dst,
	const std::vector<float>& weights,
	bool estimateScale,
	Matrix3df& outRotation,
	Vector3df& outTranslation,
	float& outScale)
{
	const size_t n = src.size();
	if (n == 0 || dst.size() != n || weights.size() != n) return false;

	double sumW = 0.0;
	for (const float w : weights) sumW += static_cast<double>(w);
	if (sumW < 1.0e-9) return false;
	const double invW = 1.0 / sumW;

	Vector3dd centroidSrc(0.0, 0.0, 0.0);
	Vector3dd centroidDst(0.0, 0.0, 0.0);
	for (size_t i = 0; i < n; ++i) {
		centroidSrc += static_cast<double>(weights[i]) * Vector3dd(src[i].x, src[i].y, src[i].z);
		centroidDst += static_cast<double>(weights[i]) * Vector3dd(dst[i].x, dst[i].y, dst[i].z);
	}
	centroidSrc *= invW;
	centroidDst *= invW;

	// Cross-covariance H = sum_i w_i * (src_i - centroidSrc) * (dst_i - centroidDst)^T.
	Matrix3dd h(0.0);
	double sigmaSrc2 = 0.0;
	for (size_t i = 0; i < n; ++i) {
		const double w = static_cast<double>(weights[i]);
		const Vector3dd a = Vector3dd(src[i].x, src[i].y, src[i].z) - centroidSrc;
		const Vector3dd b = Vector3dd(dst[i].x, dst[i].y, dst[i].z) - centroidDst;
		h += w * glm::outerProduct(a, b);
		sigmaSrc2 += w * glm::dot(a, a);
	}

	Phantom::Numerics::SVD3d svd;
	const auto svdResult = svd.calculateFullJacobi(h);
	if (!svdResult.isOk) return false;

	Matrix3dd rotation = svdResult.matrixV * glm::transpose(svdResult.matrixU);
	if (glm::determinant(rotation) < 0.0) {
		// Reflection case: flip the sign of the singular vector paired with the smallest
		// singular value (last column of V) and recompute.
		Matrix3dd correctedV = svdResult.matrixV;
		correctedV[2] = -correctedV[2];
		rotation = correctedV * glm::transpose(svdResult.matrixU);
	}

	// Umeyama scale: least-squares scalar fit of s*R*a_i to b_i given the R found above
	// (R is scale-invariant for the uniform-scale Procrustes problem, so this is exact).
	double scale = 1.0;
	if (estimateScale && sigmaSrc2 > 1.0e-12) {
		double numerator = 0.0;
		for (size_t i = 0; i < n; ++i) {
			const double w = static_cast<double>(weights[i]);
			const Vector3dd a = Vector3dd(src[i].x, src[i].y, src[i].z) - centroidSrc;
			const Vector3dd b = Vector3dd(dst[i].x, dst[i].y, dst[i].z) - centroidDst;
			numerator += w * glm::dot(rotation * a, b);
		}
		scale = numerator / sigmaSrc2;
	}

	const Vector3dd translation = centroidDst - scale * (rotation * centroidSrc);

	outRotation = Matrix3df(rotation);
	outTranslation = Vector3df(
		static_cast<float>(translation.x),
		static_cast<float>(translation.y),
		static_cast<float>(translation.z));
	outScale = static_cast<float>(scale);
	return true;
}
