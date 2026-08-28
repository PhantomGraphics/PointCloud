#include "ConcaveHull2D.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	constexpr double kTwoPi = 6.283185307179586;

	double normalizeAngle(double a) {
		while (a < 0.0) a += kTwoPi;
		while (a >= kTwoPi) a -= kTwoPi;
		return a;
	}

	bool samePoint(const Vector3df& a, const Vector3df& b) {
		return a.x == b.x && a.y == b.y;
	}

	// Orientation of the ordered triple (p,q,r): >0 counter-clockwise, <0 clockwise, 0 collinear.
	double orientation(const Vector3df& p, const Vector3df& q, const Vector3df& r) {
		return (static_cast<double>(q.x) - p.x) * (static_cast<double>(r.y) - p.y)
			- (static_cast<double>(q.y) - p.y) * (static_cast<double>(r.x) - p.x);
	}

	bool onSegment(const Vector3df& p, const Vector3df& q, const Vector3df& r) {
		return std::min(p.x, r.x) <= q.x && q.x <= std::max(p.x, r.x) &&
			std::min(p.y, r.y) <= q.y && q.y <= std::max(p.y, r.y);
	}

	// True if the open segments (a1,a2) and (b1,b2) cross. Segments sharing an endpoint (as
	// consecutive hull edges legitimately do) are never considered crossing.
	bool segmentsCross(const Vector3df& a1, const Vector3df& a2, const Vector3df& b1, const Vector3df& b2) {
		if (samePoint(a1, b1) || samePoint(a1, b2) || samePoint(a2, b1) || samePoint(a2, b2)) {
			return false;
		}
		const double d1 = orientation(b1, b2, a1);
		const double d2 = orientation(b1, b2, a2);
		const double d3 = orientation(a1, a2, b1);
		const double d4 = orientation(a1, a2, b2);
		if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))) {
			return true;
		}
		if (d1 == 0.0 && onSegment(b1, a1, b2)) return true;
		if (d2 == 0.0 && onSegment(b1, a2, b2)) return true;
		if (d3 == 0.0 && onSegment(a1, b1, a2)) return true;
		if (d4 == 0.0 && onSegment(a1, b2, a2)) return true;
		return false;
	}

	bool pointInPolygon(const std::vector<Vector3df>& polygon, const Vector3df& point) {
		const size_t n = polygon.size();
		for (size_t i = 0, j = n - 1; i < n; j = i++) {
			const auto& pi = polygon[i];
			const auto& pj = polygon[j];
			if (orientation(pi, pj, point) == 0.0 && onSegment(pi, point, pj)) {
				return true; // boundary counts as contained
			}
		}
		bool inside = false;
		for (size_t i = 0, j = n - 1; i < n; j = i++) {
			const auto& pi = polygon[i];
			const auto& pj = polygon[j];
			const bool straddles = (pi.y > point.y) != (pj.y > point.y);
			if (straddles) {
				const double xCross = (static_cast<double>(pj.x) - pi.x) * (point.y - pi.y) / (static_cast<double>(pj.y) - pi.y) + pi.x;
				if (point.x < xCross) inside = !inside;
			}
		}
		return inside;
	}

	std::vector<Vector3df> kNearest(const std::vector<Vector3df>& pool, const Vector3df& from, size_t k) {
		std::vector<std::pair<double, size_t>> withDist;
		withDist.reserve(pool.size());
		for (size_t i = 0; i < pool.size(); ++i) {
			const double dx = static_cast<double>(pool[i].x) - from.x;
			const double dy = static_cast<double>(pool[i].y) - from.y;
			withDist.emplace_back(dx * dx + dy * dy, i);
		}
		std::sort(withDist.begin(), withDist.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

		std::vector<Vector3df> result;
		result.reserve(std::min(k, withDist.size()));
		for (size_t i = 0; i < k && i < withDist.size(); ++i) {
			result.push_back(pool[withDist[i].second]);
		}
		return result;
	}

	void removeByValue(std::vector<Vector3df>& v, const Vector3df& value) {
		v.erase(std::remove_if(v.begin(), v.end(), [&](const Vector3df& p) { return samePoint(p, value); }), v.end());
	}

	// Attempts to trace a simple concave hull with a fixed neighbor count k. Returns false if no
	// closed, non-self-intersecting, all-containing hull could be found with this k.
	bool tryBuildHull(const std::vector<Vector3df>& points, size_t k, std::vector<Vector3df>& outHull) {
		size_t startIdx = 0;
		for (size_t i = 1; i < points.size(); ++i) {
			if (points[i].y < points[startIdx].y || (points[i].y == points[startIdx].y && points[i].x < points[startIdx].x)) {
				startIdx = i;
			}
		}
		const Vector3df start = points[startIdx];

		std::vector<Vector3df> dataset = points;
		removeByValue(dataset, start);

		std::vector<Vector3df> hull;
		hull.push_back(start);
		Vector3df current = start;
		double prevAngle = 0.0;
		bool closed = false;

		const size_t maxSteps = points.size() + 1;
		for (size_t step = 1; step <= maxSteps; ++step) {
			std::vector<Vector3df> pool = dataset;
			if (step >= 3) {
				bool startInPool = false;
				for (const auto& p : pool) if (samePoint(p, start)) { startInPool = true; break; }
				if (!startInPool) pool.push_back(start);
			}
			if (pool.empty()) {
				return false;
			}

			const auto neighbors = kNearest(pool, current, std::min(k, pool.size()));

			// Prefer the candidate requiring the smallest counter-clockwise rotation from the
			// previous edge's direction -- this is what makes the trace hug the boundary of the
			// point set (the gift-wrapping selection rule) instead of cutting across it. Sorting
			// by the opposite (clockwise) turn descending looks equivalent at first glance but
			// is NOT: (360-x) mod 360 has a discontinuity exactly at x=0 (the all-important
			// "continue straight ahead" case), which silently breaks the ordering.
			std::vector<std::pair<double, Vector3df>> byTurn;
			byTurn.reserve(neighbors.size());
			for (const auto& c : neighbors) {
				const double candidateAngle = std::atan2(static_cast<double>(c.y) - current.y, static_cast<double>(c.x) - current.x);
				const double turn = normalizeAngle(candidateAngle - prevAngle);
				byTurn.emplace_back(turn, c);
			}
			std::sort(byTurn.begin(), byTurn.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

			bool found = false;
			Vector3df selected;
			for (const auto& [turn, candidate] : byTurn) {
				if (samePoint(candidate, start) && step < 3) {
					continue; // don't allow closing before a minimal triangle exists
				}
				bool intersects = false;
				// Skip the edge immediately preceding `current` (index hull.size()-2 -> hull.size()-1):
				// it shares the vertex `current` with the candidate edge, which segmentsCross already
				// treats as non-crossing, so there's nothing to gain by checking it explicitly.
				for (size_t i = 0; i + 2 < hull.size(); ++i) {
					if (segmentsCross(current, candidate, hull[i], hull[i + 1])) {
						intersects = true;
						break;
					}
				}
				if (!intersects) {
					selected = candidate;
					found = true;
					break;
				}
			}

			if (!found) {
				return false;
			}

			prevAngle = std::atan2(static_cast<double>(selected.y) - current.y, static_cast<double>(selected.x) - current.x);
			current = selected;
			hull.push_back(current);
			removeByValue(dataset, current);

			if (samePoint(current, start)) {
				closed = true;
				break;
			}
		}

		if (!closed || hull.size() < 4) { // hull includes the closing repeat of `start`
			return false;
		}
		hull.pop_back(); // drop the duplicated closing vertex

		for (const auto& p : points) {
			if (!pointInPolygon(hull, p)) {
				return false;
			}
		}

		outHull = hull;
		return true;
	}
}

bool ConcaveHull2D::compute(size_t k, size_t maxK)
{
	hullPoints.clear();
	area = 0.0f;

	std::vector<Vector3df> unique = pointCloud;
	std::sort(unique.begin(), unique.end(), [](const Vector3df& a, const Vector3df& b) {
		if (a.x != b.x) return a.x < b.x;
		return a.y < b.y;
	});
	unique.erase(std::unique(unique.begin(), unique.end(), samePoint), unique.end());

	if (unique.size() < 3) {
		return false;
	}

	std::vector<Vector3df> hull;
	if (unique.size() == 3) {
		hull = unique;
		if (orientation(hull[0], hull[1], hull[2]) < 0.0) {
			std::swap(hull[1], hull[2]);
		}
	} else {
		const size_t upperBound = (maxK == 0) ? (unique.size() - 1) : std::min(maxK, unique.size() - 1);
		size_t currentK = std::max<size_t>(3, std::min(k, upperBound));

		bool success = false;
		for (; currentK <= upperBound; ++currentK) {
			if (::tryBuildHull(unique, currentK, hull)) {
				success = true;
				break;
			}
		}
		if (!success) {
			return false;
		}
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
