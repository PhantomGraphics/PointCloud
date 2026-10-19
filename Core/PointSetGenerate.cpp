#include "PointSetGenerate.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	// SplitMix64: tiny, fixed by definition, good enough for sampling.
	struct SplitMix64
	{
		uint64_t state;
		uint64_t next()
		{
			uint64_t z = (state += 0x9E3779B97F4A7C15ull);
			z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
			z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
			return z ^ (z >> 31);
		}
		// Uniform in [0, 1) with 53 bits.
		double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
	};

	double areaOf(const SurfaceTriangle& t)
	{
		const glm::dvec3 ab = glm::dvec3(t.b) - glm::dvec3(t.a);
		const glm::dvec3 ac = glm::dvec3(t.c) - glm::dvec3(t.a);
		return 0.5 * glm::length(glm::cross(ab, ac));
	}

}

double Phantom::PC::totalArea(const std::vector<SurfaceTriangle>& triangles)
{
	double sum = 0.0;
	for (const auto& t : triangles) sum += areaOf(t);
	return sum;
}

PointSet Phantom::PC::pointsFromVertices(const std::vector<Vector3df>& vertices, const Vector3df& color, float radius)
{
	PointSet set;
	set.positions = vertices;
	set.colors.assign(vertices.size(), color);
	set.radii.assign(vertices.size(), radius);
	return set;
}

PointSet Phantom::PC::samplePointsOnSurface(const std::vector<SurfaceTriangle>& triangles, size_t count, uint64_t seed,
	const Vector3df& color, float radius)
{
	PointSet set;
	if (count == 0 || triangles.empty()) return set;

	std::vector<double> cumulative(triangles.size());
	double sum = 0.0;
	for (size_t i = 0; i < triangles.size(); ++i) {
		const double a = areaOf(triangles[i]);
		sum += std::isfinite(a) && a > 0.0 ? a : 0.0;
		cumulative[i] = sum;
	}
	if (!(sum > 0.0)) return set;

	set.positions.reserve(count);
	set.colors.assign(count, color);
	set.radii.assign(count, radius);
	set.normals.reserve(count);
	SplitMix64 rng{ seed };
	for (size_t k = 0; k < count; ++k) {
		const double pick = rng.uniform() * sum;
		size_t i = static_cast<size_t>(std::upper_bound(cumulative.begin(), cumulative.end(), pick) - cumulative.begin());
		if (i >= triangles.size()) i = triangles.size() - 1;
		// A triangle with no area has the same cumulative value as its predecessor and is never chosen by upper_bound; step over
		// the rare equal-value case anyway.
		while (i + 1 < triangles.size() && areaOf(triangles[i]) <= 0.0) ++i;

		const SurfaceTriangle& t = triangles[i];
		const double r1 = rng.uniform(), r2 = rng.uniform();
		const double s = std::sqrt(r1);
		const double wa = 1.0 - s, wb = s * (1.0 - r2), wc = s * r2;
		const glm::dvec3 p = wa * glm::dvec3(t.a) + wb * glm::dvec3(t.b) + wc * glm::dvec3(t.c);
		set.positions.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
		glm::vec3 n = glm::cross(t.b - t.a, t.c - t.a);
		const float len = glm::length(n);
		set.normals.push_back(len > 0.f ? n / len : glm::vec3(0.f, 1.f, 0.f));
	}
	return set;
}
