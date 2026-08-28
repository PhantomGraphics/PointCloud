#include "ConvexHull2D.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	// 2D cross product of (A-O) x (B-O), in double precision to avoid accumulated float error
	// across the many comparisons the monotone chain performs. Positive means O->A->B turns
	// left (counter-clockwise).
	double cross2d(const Vector3df& o, const Vector3df& a, const Vector3df& b) {
		const double ax = static_cast<double>(a.x) - o.x;
		const double ay = static_cast<double>(a.y) - o.y;
		const double bx = static_cast<double>(b.x) - o.x;
		const double by = static_cast<double>(b.y) - o.y;
		return ax * by - ay * bx;
	}
}

bool ConvexHull2D::compute()
{
	hullPoints.clear();
	area = 0.0f;

	std::vector<Vector3df> sorted = pointCloud;
	std::sort(sorted.begin(), sorted.end(), [](const Vector3df& a, const Vector3df& b) {
		if (a.x != b.x) return a.x < b.x;
		return a.y < b.y;
	});
	sorted.erase(std::unique(sorted.begin(), sorted.end(), [](const Vector3df& a, const Vector3df& b) {
		return a.x == b.x && a.y == b.y;
	}), sorted.end());

	if (sorted.size() < 3) {
		return false;
	}

	// Andrew's monotone chain: build the lower and upper hulls separately, each keeping only
	// left turns (cross > 0), then splice them together (each half's last point is dropped to
	// avoid repeating the other half's first point).
	std::vector<Vector3df> lower;
	for (const auto& p : sorted) {
		while (lower.size() >= 2 && cross2d(lower[lower.size() - 2], lower[lower.size() - 1], p) <= 0.0) {
			lower.pop_back();
		}
		lower.push_back(p);
	}

	std::vector<Vector3df> upper;
	for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
		while (upper.size() >= 2 && cross2d(upper[upper.size() - 2], upper[upper.size() - 1], *it) <= 0.0) {
			upper.pop_back();
		}
		upper.push_back(*it);
	}

	lower.pop_back();
	upper.pop_back();

	std::vector<Vector3df> hull = lower;
	hull.insert(hull.end(), upper.begin(), upper.end());

	if (hull.size() < 3) {
		return false; // all input points were collinear
	}

	hullPoints.reserve(hull.size());
	for (const auto& p : hull) {
		hullPoints.emplace_back(p.x, p.y, 0.0f);
	}

	double shoelace = 0.0;
	for (size_t i = 0; i < hullPoints.size(); ++i) {
		const auto& p0 = hullPoints[i];
		const auto& p1 = hullPoints[(i + 1) % hullPoints.size()];
		shoelace += static_cast<double>(p0.x) * p1.y - static_cast<double>(p1.x) * p0.y;
	}
	area = static_cast<float>(std::abs(shoelace) * 0.5);

	return true;
}
