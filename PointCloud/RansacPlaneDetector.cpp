#include "RansacPlaneDetector.h"

#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace Phantom::PC;
using namespace Phantom::Math;

bool RansacPlaneDetector::detect(
	const std::vector<Vector3df>& points,
	PlaneModel& bestModel,
	int iterations,
	float distThreshold,
	size_t minInliers)
{
	if (points.size() < 3) return false;

	const unsigned baseSeed = hasSeed ? rngSeed : std::random_device{}();
	size_t bestInlierCount = 0;

	// Each iteration builds its own RNG seeded from `it` instead of sharing one std::mt19937
	// across threads (racing on its internal state) or seeding per-thread (making results depend
	// on the thread count) -- this keeps detect() reproducible for a given seed regardless of how
	// many threads run it.
#pragma omp parallel for
	for (int it = 0; it < iterations; ++it) {
		std::mt19937 rng(baseSeed + static_cast<unsigned>(it));
		std::uniform_int_distribution<size_t> uid(0, points.size() - 1);

		// ランダムに3点を選ぶ（重複禁止）
		std::vector<size_t> sampleIdx;
		sampleIdx.reserve(3);
		while (sampleIdx.size() < 3) {
			auto idx = uid(rng);
			if (std::find(sampleIdx.begin(), sampleIdx.end(), idx) == sampleIdx.end()) {
				sampleIdx.push_back(idx);
			}
		}

		const auto& p0 = points[sampleIdx[0]];
		const auto& p1 = points[sampleIdx[1]];
		const auto& p2 = points[sampleIdx[2]];

		Vector3df normal;
		float offset = 0.0f;
		if (!computePlaneFromThreePoints(p0, p1, p2, normal, offset)) {
			continue; // 劣悪な三点（同一直線等）
		}

		// 全点について平面からの距離を計算してインライア判定
		std::vector<size_t> inliers;
		inliers.reserve(points.size());
		for (size_t i = 0; i < points.size(); ++i) {
			const float d = pointPlaneDistance(points[i], normal, offset);
			if (d <= distThreshold) {
				inliers.push_back(i);
			}
		}

		// Compare-and-update must happen atomically together; reading bestInlierCount outside the
		// critical section (as a fast-path guard) would be a data race with the writes below.
		if (inliers.size() >= minInliers) {
#pragma omp critical
			{
				if (inliers.size() > bestInlierCount) {
					bestInlierCount = inliers.size();
					bestModel.normal = normal;
					bestModel.offset = offset;
					bestModel.inliers = std::move(inliers);
				}
			}
		}
	}

	return bestInlierCount >= minInliers;
}

Vector3df RansacPlaneDetector::computeCentroid(const std::vector<Vector3df>& pts, const std::vector<size_t>& indices) const
{
	Vector3df c(0.0f);
	for (auto i : indices) {
		c += pts[i];
	}
	c /= static_cast<float>(indices.size());
	return c;
}

bool RansacPlaneDetector::computePlaneFromThreePoints(const Vector3df& p0, const Vector3df& p1, const Vector3df& p2, Vector3df& normal, float& offset) const
{
	const auto v1 = p1 - p0;
	const auto v2 = p2 - p0;
	const auto n = glm::cross(v1, v2);
	const float nlen = glm::length(n);
	if (nlen < 1e-6f) return false;
	normal = n / nlen;
	offset = -glm::dot(normal, p0); // n·x + offset = 0
	return true;
}

float RansacPlaneDetector::pointPlaneDistance(const Vector3df& p, const Vector3df& normal, float offset) const
{
	return std::fabs(glm::dot(normal, p) + offset);
}