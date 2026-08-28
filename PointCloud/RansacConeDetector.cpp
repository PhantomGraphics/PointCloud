#include "RansacConeDetector.h"

#include <random>
#include <algorithm>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

Vector3df RansacConeDetector::computeCentroid(const std::vector<Vector3df>& pts, const std::vector<size_t>& indices) const
{
	Vector3df c(0.0f);
	for (auto i : indices) {
		c += pts[i];
	}
	c /= static_cast<float>(indices.size());
	return c;
}

Vector3df RansacConeDetector::estimateDominantDirection(const std::vector<Vector3df>& pts, const std::vector<size_t>& indices) const
{
	float cov00 = 0, cov01 = 0, cov02 = 0;
	float cov11 = 0, cov12 = 0, cov22 = 0;

	const auto centroid = computeCentroid(pts, indices);
	for (auto i : indices) {
		const auto v = pts[i] - centroid;
		cov00 += v.x * v.x; cov01 += v.x * v.y; cov02 += v.x * v.z;
		cov11 += v.y * v.y; cov12 += v.y * v.z; cov22 += v.z * v.z;
	}

	auto mulCov = [&](const Vector3df& b) -> Vector3df {
		return Vector3df(
			cov00 * b.x + cov01 * b.y + cov02 * b.z,
			cov01 * b.x + cov11 * b.y + cov12 * b.z,
			cov02 * b.x + cov12 * b.y + cov22 * b.z);
	};

	// Power iteration converges to the dominant (largest-eigenvalue) eigenvector, which for a
	// cone sample elongated along its axis approximates the axis direction (sign ambiguous;
	// resolved later from the sign of the fitted radius-vs-distance slope).
	Vector3df v1(1.0f, 0.0f, 0.0f);
	for (int iter = 0; iter < 30; ++iter) {
		const Vector3df nv = mulCov(v1);
		const float len = glm::length(nv);
		if (len < 1.0e-6f) break;
		v1 = nv / len;
	}
	return v1;
}

bool RansacConeDetector::fitConeFromDirection(
	const std::vector<Vector3df>& pts, const std::vector<size_t>& indices,
	const Vector3df& roughDirection,
	Vector3df& apex, Vector3df& axis, float& halfAngleRad) const
{
	const Vector3df dir = glm::normalize(roughDirection);
	const auto centroid = computeCentroid(pts, indices);

	// A cone's radius grows linearly with distance along its axis from the apex: fit
	// radial = m*axial + b via least squares over the sample, using an arbitrary (unresolved)
	// sign for `dir` as the projection axis.
	const size_t n = indices.size();
	double sumS = 0.0, sumR = 0.0, sumSS = 0.0, sumSR = 0.0;
	for (auto i : indices) {
		const Vector3df v = pts[i] - centroid;
		const double s = glm::dot(v, dir);
		const double r = glm::length(v - dir * static_cast<float>(s));
		sumS += s; sumR += r; sumSS += s * s; sumSR += s * r;
	}

	const double denom = static_cast<double>(n) * sumSS - sumS * sumS;
	if (std::fabs(denom) < 1.0e-9) {
		return false; // Degenerate axial spread (e.g. all sample points at the same s).
	}

	const double m = (static_cast<double>(n) * sumSR - sumS * sumR) / denom;
	if (std::fabs(m) < 1.0e-6) {
		return false; // Near-zero slope: radius doesn't grow along this axis (not a cone).
	}
	const double b = (sumR - m * sumS) / static_cast<double>(n);

	// The radius==0 crossing of the fitted line, in the (arbitrary-sign) `dir` coordinate.
	const double apexS = -b / m;
	apex = centroid + dir * static_cast<float>(apexS);
	// Flip so `axis` always points from the apex toward increasing radius (the base direction).
	axis = (m >= 0.0) ? dir : -dir;
	halfAngleRad = std::atan(static_cast<float>(std::fabs(m)));
	return true;
}

float RansacConeDetector::pointLineDistance(const Vector3df& p, const Vector3df& origin, const Vector3df& dir) const
{
	const auto v = p - origin;
	const float proj = glm::dot(v, dir);
	const auto perp = v - dir * proj;
	return glm::length(perp);
}

bool RansacConeDetector::detect(
	const std::vector<Vector3df>& points,
	ConeModel& bestModel,
	int iterations,
	float distThreshold,
	size_t minInliers)
{
	if (points.size() < 6) return false;

	const unsigned baseSeed = hasSeed ? rngSeed : std::random_device{}();

	// A modest (not minimal, not huge) sample size: large enough that the sample covariance's
	// dominant-eigenvalue margin (axial vs. lateral variance -- see estimateDominantDirection)
	// reliably reflects the true axis rather than sampling noise, but small enough that a
	// meaningful fraction of samples stay outlier-free at a moderate outlier ratio (outliers are
	// spread over a much larger volume than the true surface, so even one can measurably skew a
	// small sample's covariance-based direction estimate).
	const size_t sampleSize = std::min(points.size(), size_t(20));

	size_t bestInlierCount = 0;

	// See RansacPlaneDetector::detect() for why each iteration gets its own RNG seeded from `it`.
#pragma omp parallel for
	for (int it = 0; it < iterations; ++it) {
		std::mt19937 rng(baseSeed + static_cast<unsigned>(it));
		std::uniform_int_distribution<size_t> uid(0, points.size() - 1);

		std::vector<size_t> sampleIdx;
		sampleIdx.reserve(sampleSize);
		while (sampleIdx.size() < sampleSize) {
			auto idx = uid(rng);
			if (std::find(sampleIdx.begin(), sampleIdx.end(), idx) == sampleIdx.end()) {
				sampleIdx.push_back(idx);
			}
		}

		const auto roughDir = estimateDominantDirection(points, sampleIdx);
		if (glm::length(roughDir) < 1.0e-6f) continue;

		Vector3df apex, axis;
		float halfAngle = 0.0f;
		if (!fitConeFromDirection(points, sampleIdx, roughDir, apex, axis, halfAngle)) continue;
		// Reject near-cylindrical (angle ~0) and near-planar (angle ~90deg) degenerate fits.
		if (halfAngle < 1.0e-3f || halfAngle > 1.55f) continue;

		const float tanHalf = std::tan(halfAngle);

		std::vector<size_t> inliers;
		inliers.reserve(points.size());
		for (size_t i = 0; i < points.size(); ++i) {
			const Vector3df v = points[i] - apex;
			const float axial = glm::dot(v, axis);
			const float radial = pointLineDistance(points[i], apex, axis);
			const float expectedRadial = std::max(0.0f, axial) * tanHalf;
			if (std::fabs(radial - expectedRadial) <= distThreshold) {
				inliers.push_back(i);
			}
		}

		if (inliers.size() >= minInliers) {
#pragma omp critical
			{
				if (inliers.size() > bestInlierCount) {
					bestInlierCount = inliers.size();
					bestModel.apex = apex;
					bestModel.axis = axis;
					bestModel.halfAngleRad = halfAngle;
					bestModel.inliers = std::move(inliers);
				}
			}
		}
	}

	return bestInlierCount >= minInliers;
}
