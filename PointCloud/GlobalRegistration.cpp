#include "GlobalRegistration.h"
#include "ICPRegistration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

using namespace Phantom::PC;
using namespace Phantom::Math;

float GlobalRegistration::featureDistanceSq(const FPFHEstimator::Histogram& a, const FPFHEstimator::Histogram& b)
{
	float sum = 0.0f;
	for (int i = 0; i < FPFHEstimator::HistogramSize; ++i) {
		const float d = a[i] - b[i];
		sum += d * d;
	}
	return sum;
}

std::vector<size_t> GlobalRegistration::matchFeatures(
	const std::vector<FPFHEstimator::Histogram>& sourceFeatures,
	const std::vector<FPFHEstimator::Histogram>& targetFeatures)
{
	std::vector<size_t> matches(sourceFeatures.size(), 0);
	const int n = static_cast<int>(sourceFeatures.size());
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		size_t best = 0;
		float bestDist = std::numeric_limits<float>::max();
		for (size_t j = 0; j < targetFeatures.size(); ++j) {
			const float d = featureDistanceSq(sourceFeatures[static_cast<size_t>(i)], targetFeatures[j]);
			if (d < bestDist) {
				bestDist = d;
				best = j;
			}
		}
		matches[static_cast<size_t>(i)] = best;
	}
	return matches;
}

bool GlobalRegistration::align(
	const std::vector<Vector3df>& source,
	const std::vector<FPFHEstimator::Histogram>& sourceFeatures,
	const std::vector<Vector3df>& target,
	const std::vector<FPFHEstimator::Histogram>& targetFeatures,
	Result& result,
	int iterations,
	float maxCorrespondenceDistance,
	size_t sampleSize,
	float edgeLengthTolerance,
	size_t minInliers) const
{
	result = Result{};
	if (source.size() != sourceFeatures.size() || target.size() != targetFeatures.size()) return false;
	if (source.empty() || target.empty()) return false;
	if (sampleSize < 3 || source.size() < sampleSize || iterations <= 0) return false;

	// Correspondence set is fixed once up front (one-way nearest neighbor in feature space); the
	// RANSAC loop below only ever tests subsets of *this* set for 3D geometric consistency, it
	// never re-matches features per iteration.
	const std::vector<size_t> matched = matchFeatures(sourceFeatures, targetFeatures);
	const size_t n = source.size();

	const unsigned baseSeed = hasSeed ? rngSeed : std::random_device{}();

	const float maxDistSq = maxCorrespondenceDistance * maxCorrespondenceDistance;
	const float edgeRatioLow = 1.0f - edgeLengthTolerance;
	const float edgeRatioHigh = 1.0f / edgeRatioLow;

	size_t bestInlierCount = 0;
	Matrix3df bestRotation = identitiyMatrix3d<float>();
	Vector3df bestTranslation(0.0f, 0.0f, 0.0f);
	bool foundAny = false;

	// See RansacPlaneDetector::detect() for why each iteration gets its own RNG seeded from `it`
	// (also avoids sharing the `sampleIdx` scratch buffer across threads).
#pragma omp parallel for
	for (int it = 0; it < iterations; ++it) {
		std::mt19937 rng(baseSeed + static_cast<unsigned>(it));
		std::uniform_int_distribution<size_t> uid(0, n - 1);

		std::vector<size_t> sampleIdx;
		sampleIdx.reserve(sampleSize);
		while (sampleIdx.size() < sampleSize) {
			const size_t idx = uid(rng);
			if (std::find(sampleIdx.begin(), sampleIdx.end(), idx) == sampleIdx.end()) {
				sampleIdx.push_back(idx);
			}
		}

		// Cheap geometric-consistency pruning before fitting (Open3D's "correspondence checker"
		// idea): reject the sample outright if any pair's source/target edge length disagrees by
		// more than `edgeLengthTolerance`, since most random samples over a noisy feature-space
		// correspondence set are wrong and this is far cheaper than the full O(N) inlier count.
		bool consistent = true;
		for (size_t a = 0; a < sampleIdx.size() && consistent; ++a) {
			for (size_t b = a + 1; b < sampleIdx.size(); ++b) {
				const float srcLen = glm::length(source[sampleIdx[a]] - source[sampleIdx[b]]);
				const float dstLen = glm::length(target[matched[sampleIdx[a]]] - target[matched[sampleIdx[b]]]);
				if (srcLen < 1.0e-6f || dstLen < 1.0e-6f) {
					consistent = false;
					break;
				}
				const float ratio = srcLen / dstLen;
				if (ratio < edgeRatioLow || ratio > edgeRatioHigh) {
					consistent = false;
					break;
				}
			}
		}
		if (!consistent) continue;

		std::vector<Vector3df> sampleSrc, sampleDst;
		sampleSrc.reserve(sampleIdx.size());
		sampleDst.reserve(sampleIdx.size());
		for (const size_t idx : sampleIdx) {
			sampleSrc.push_back(source[idx]);
			sampleDst.push_back(target[matched[idx]]);
		}
		const std::vector<float> sampleWeights(sampleIdx.size(), 1.0f);

		Matrix3df rotation;
		Vector3df translation;
		float scale = 1.0f;
		if (!ICPRegistration::computeRigidTransform(sampleSrc, sampleDst, sampleWeights, false, rotation, translation, scale)) {
			continue;
		}

		size_t inlierCount = 0;
		for (size_t i = 0; i < n; ++i) {
			const Vector3df transformed = rotation * source[i] + translation;
			const Vector3df diff = transformed - target[matched[i]];
			if (glm::dot(diff, diff) <= maxDistSq) {
				++inlierCount;
			}
		}

		// No cheap pre-filter is available here (unlike the Ransac*Detector classes' minInliers
		// guard), so the compare-and-update always goes through the critical section; the O(n)
		// inlier count above dominates the per-iteration cost, so this stays cheap relative to it.
#pragma omp critical
		{
			if (inlierCount > bestInlierCount) {
				bestInlierCount = inlierCount;
				bestRotation = rotation;
				bestTranslation = translation;
				foundAny = true;
			}
		}
	}

	if (!foundAny || bestInlierCount < minInliers) return false;

	// Final refit using every inlier correspondence under the RANSAC winner (rather than just its
	// minimal sample) for a more accurate transform, mirroring PCL's SampleConsensusInitialAlignment.
	std::vector<Vector3df> inlierSrc, inlierDst;
	inlierSrc.reserve(bestInlierCount);
	inlierDst.reserve(bestInlierCount);
	for (size_t i = 0; i < n; ++i) {
		const Vector3df transformed = bestRotation * source[i] + bestTranslation;
		const Vector3df diff = transformed - target[matched[i]];
		if (glm::dot(diff, diff) <= maxDistSq) {
			inlierSrc.push_back(source[i]);
			inlierDst.push_back(target[matched[i]]);
		}
	}

	Matrix3df finalRotation = bestRotation;
	Vector3df finalTranslation = bestTranslation;
	if (inlierSrc.size() >= 3) {
		Matrix3df refitRotation;
		Vector3df refitTranslation;
		float refitScale = 1.0f;
		const std::vector<float> inlierWeights(inlierSrc.size(), 1.0f);
		// Refit failure (e.g. near-degenerate inlier geometry) just falls back to the RANSAC
		// winner rather than failing the whole call.
		if (ICPRegistration::computeRigidTransform(inlierSrc, inlierDst, inlierWeights, false, refitRotation, refitTranslation, refitScale)) {
			finalRotation = refitRotation;
			finalTranslation = refitTranslation;
		}
	}

	size_t finalInlierCount = 0;
	double finalSumSq = 0.0;
	for (size_t i = 0; i < n; ++i) {
		const Vector3df transformed = finalRotation * source[i] + finalTranslation;
		const Vector3df diff = transformed - target[matched[i]];
		const float sqDist = glm::dot(diff, diff);
		if (sqDist <= maxDistSq) {
			++finalInlierCount;
			finalSumSq += static_cast<double>(sqDist);
		}
	}

	result.rotation = finalRotation;
	result.translation = finalTranslation;
	result.inlierCount = finalInlierCount;
	result.inlierRmse = finalInlierCount > 0
		? std::sqrt(static_cast<float>(finalSumSq / static_cast<double>(finalInlierCount)))
		: 0.0f;
	return true;
}
