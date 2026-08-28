#include "RansacCylinderDetector.h"
#include <limits>
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

bool RansacCylinderDetector::detect(
	const std::vector<Vector3df>& points,
	CylinderModel& bestModel,
	int iterations,
	float distThreshold,
	size_t minInliers)
{
	if (points.size() < 3) return false;

	const unsigned baseSeed = 123456789u;

	// Use more points for direction estimation; 3 points are not reliable enough.
	const size_t dirSampleSize = std::min(points.size(), size_t(20));

	size_t bestInlierCount = 0;

	// See RansacPlaneDetector::detect() for why each iteration gets its own RNG seeded from `it`.
#pragma omp parallel for
	for (int it = 0; it < iterations; ++it) {
		std::mt19937 rng(baseSeed + static_cast<unsigned>(it));
		std::uniform_int_distribution<size_t> uid(0, points.size() - 1);

		// Sample multiple points for axis direction estimation.
		std::vector<size_t> sampleIdx;
		sampleIdx.reserve(dirSampleSize);
		while (sampleIdx.size() < dirSampleSize) {
			auto idx = uid(rng);
			if (std::find(sampleIdx.begin(), sampleIdx.end(), idx) == sampleIdx.end()) {
				sampleIdx.push_back(idx);
			}
		}

		// Estimate centroid and principal direction from the sampled points.
		auto centroid = computeCentroid(points, sampleIdx);
		auto dir = estimatePrincipalDirection(points, sampleIdx);
		// Skip degenerate direction.
		const float dirLen = glm::length(dir);
		if (dirLen < 1e-6f) continue;
		dir = glm::normalize(dir);

		// Use the median radial distance of sampled points as the radius estimate.
		std::vector<float> dists;
		dists.reserve(sampleIdx.size());
		for (auto i : sampleIdx) {
			dists.push_back(pointLineDistance(points[i], centroid, dir));
		}
		std::nth_element(dists.begin(), dists.begin() + dists.size() / 2, dists.end());
		const float radius = dists[dists.size() / 2];

		// Count inliers over all points.
		std::vector<size_t> inliers;
		inliers.reserve(points.size());
		for (size_t i = 0; i < points.size(); ++i) {
			const float d = std::abs(pointLineDistance(points[i], centroid, dir) - radius);
			if (d <= distThreshold) {
				inliers.push_back(i);
			}
		}

		if (inliers.size() >= minInliers) {
#pragma omp critical
			{
				if (inliers.size() > bestInlierCount) {
					bestInlierCount = inliers.size();
					bestModel.origin = centroid;
					bestModel.direction = dir;
					bestModel.radius = radius;
					bestModel.inliers = std::move(inliers);
				}
			}
		}
	}

	return bestInlierCount >= minInliers;
}

Vector3df RansacCylinderDetector::computeCentroid(const std::vector<Vector3df>& pts, const std::vector<size_t>& indices) const
{
	Vector3df c(0.0f);
	for (auto i : indices) {
		c += pts[i];
	}
	c /= static_cast<float>(indices.size());
	return c;
}

Vector3df RansacCylinderDetector::estimatePrincipalDirection(const std::vector<Vector3df>& pts, const std::vector<size_t>& indices) const
{
	// Build the 3x3 covariance matrix.
	float cov00 = 0, cov01 = 0, cov02 = 0;
	float cov11 = 0, cov12 = 0, cov22 = 0;

	auto centroid = computeCentroid(pts, indices);

	for (auto i : indices) {
		const auto v = pts[i] - centroid;
		cov00 += v.x * v.x;
		cov01 += v.x * v.y;
		cov02 += v.x * v.z;
		cov11 += v.y * v.y;
		cov12 += v.y * v.z;
		cov22 += v.z * v.z;
	}

	// Helper: multiply the symmetric covariance matrix by a vector.
	auto mulCov = [&](const Vector3df& b) -> Vector3df {
		return Vector3df(
			cov00 * b.x + cov01 * b.y + cov02 * b.z,
			cov01 * b.x + cov11 * b.y + cov12 * b.z,
			cov02 * b.x + cov12 * b.y + cov22 * b.z);
	};

	// Find the dominant eigenvector v1 (largest eigenvalue) via power iteration.
	Vector3df v1(1.0f, 0.0f, 0.0f);
	float lambda1 = 0.0f;
	for (int iter = 0; iter < 30; ++iter) {
		const Vector3df nv = mulCov(v1);
		const float len = glm::length(nv);
		if (len < 1e-6f) break;
		lambda1 = len;
		v1 = nv / len;
	}

	// Deflation: find the second eigenvector v2.
	// Applying (A - lambda1 * v1*v1^T) removes the v1 component so power
	// iteration converges to the next largest eigenvector.
	Vector3df v2(0.0f, 1.0f, 0.0f);
	for (int iter = 0; iter < 30; ++iter) {
		const Vector3df nv = mulCov(v2) - lambda1 * glm::dot(v1, v2) * v1;
		const float len = glm::length(nv);
		if (len < 1e-6f) break;
		v2 = nv / len;
	}

	// The cylinder axis is the direction of minimum variance, which is
	// perpendicular to the two dominant eigenvectors (their cross product).
	return glm::normalize(glm::cross(v1, v2));
}

float RansacCylinderDetector::pointLineDistance(const Vector3df& p, const Vector3df& origin, const Vector3df& dir) const
{
	const auto v = p - origin;
	const float proj = glm::dot(v, dir);
	const auto perp = v - dir * proj;
	return glm::length(perp);
}
