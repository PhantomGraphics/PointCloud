#include "RansacSphereDetector.h"

#include <random>
#include <algorithm>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

bool RansacSphereDetector::computeSphereFromFourPoints(
	const Vector3df& p0, const Vector3df& p1,
	const Vector3df& p2, const Vector3df& p3,
	Vector3df& center, float& radius) const
{
	// From |p_k-c|^2 = |p0-c|^2: 2*(p_k-p0)*c = |p_k|^2-|p0|^2, i.e. d_k.c = e_k for k=1,2,3
	// (linear in the unknown center c, since the |c|^2 terms cancel).
	const float sq0 = glm::dot(p0, p0);
	const Vector3df d1 = p1 - p0, d2 = p2 - p0, d3 = p3 - p0;
	const float e1 = 0.5f * (glm::dot(p1, p1) - sq0);
	const float e2 = 0.5f * (glm::dot(p2, p2) - sq0);
	const float e3 = 0.5f * (glm::dot(p3, p3) - sq0);

	// Solved via reciprocal vectors (equivalent to Cramer's rule for a system given by row
	// vectors d1,d2,d3): c = (e1*(d2xd3) + e2*(d3xd1) + e3*(d1xd2)) / (d1.(d2xd3)). Verified by
	// dotting both sides with d1/d2/d3 and using the cyclic invariance of the scalar triple
	// product (a.(bxc) = b.(cxa) = c.(axb)).
	const Vector3df d2xd3 = glm::cross(d2, d3);
	const float volume = glm::dot(d1, d2xd3);
	if (std::fabs(volume) < 1.0e-9f) {
		return false; // (near-)coplanar points: no unique sphere.
	}

	const Vector3df d3xd1 = glm::cross(d3, d1);
	const Vector3df d1xd2 = glm::cross(d1, d2);
	center = (e1 * d2xd3 + e2 * d3xd1 + e3 * d1xd2) / volume;
	radius = glm::length(p0 - center);
	return true;
}

bool RansacSphereDetector::detect(
	const std::vector<Vector3df>& points,
	SphereModel& bestModel,
	int iterations,
	float distThreshold,
	size_t minInliers)
{
	if (points.size() < 4) return false;

	const unsigned baseSeed = hasSeed ? rngSeed : std::random_device{}();
	size_t bestInlierCount = 0;

	// See RansacPlaneDetector::detect() for why each iteration gets its own RNG seeded from `it`.
#pragma omp parallel for
	for (int it = 0; it < iterations; ++it) {
		std::mt19937 rng(baseSeed + static_cast<unsigned>(it));
		std::uniform_int_distribution<size_t> uid(0, points.size() - 1);

		std::vector<size_t> sampleIdx;
		sampleIdx.reserve(4);
		while (sampleIdx.size() < 4) {
			auto idx = uid(rng);
			if (std::find(sampleIdx.begin(), sampleIdx.end(), idx) == sampleIdx.end()) {
				sampleIdx.push_back(idx);
			}
		}

		Vector3df center;
		float radius = 0.0f;
		if (!computeSphereFromFourPoints(
			points[sampleIdx[0]], points[sampleIdx[1]], points[sampleIdx[2]], points[sampleIdx[3]],
			center, radius)) {
			continue;
		}
		if (radius < 1.0e-6f) continue;

		std::vector<size_t> inliers;
		inliers.reserve(points.size());
		for (size_t i = 0; i < points.size(); ++i) {
			const float d = std::fabs(glm::length(points[i] - center) - radius);
			if (d <= distThreshold) {
				inliers.push_back(i);
			}
		}

		if (inliers.size() >= minInliers) {
#pragma omp critical
			{
				if (inliers.size() > bestInlierCount) {
					bestInlierCount = inliers.size();
					bestModel.center = center;
					bestModel.radius = radius;
					bestModel.inliers = std::move(inliers);
				}
			}
		}
	}

	return bestInlierCount >= minInliers;
}
