#include "PointSetPick.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace Phantom::PC;

namespace {

	constexpr float kNear = 1.0e-4f;   // view-space distance below which a point / vertex counts as behind the eye

	// Nearest view-space distance (w) per pixel of a rectangle of the viewport.
	class DepthMap
	{
	public:
		DepthMap(int x0, int y0, int x1, int y1)
			: x0_(x0), y0_(y0), w_(std::max(0, x1 - x0)), h_(std::max(0, y1 - y0)),
			depth_(static_cast<size_t>(w_) * static_cast<size_t>(h_), std::numeric_limits<float>::infinity())
		{
		}

		bool contains(int x, int y) const { return x >= x0_ && y >= y0_ && x < x0_ + w_ && y < y0_ + h_; }

		float at(int x, int y) const
		{
			return contains(x, y) ? depth_[static_cast<size_t>(y - y0_) * w_ + (x - x0_)] : std::numeric_limits<float>::infinity();
		}

		void write(int x, int y, float d)
		{
			float& cell = depth_[static_cast<size_t>(y - y0_) * w_ + (x - x0_)];
			if (d < cell) cell = d;
		}

		// A flat disc of the given view-space distance.
		void splat(float cx, float cy, float radiusPx, float d)
		{
			const int xa = std::max(x0_, static_cast<int>(std::floor(cx - radiusPx)));
			const int xb = std::min(x0_ + w_ - 1, static_cast<int>(std::ceil(cx + radiusPx)));
			const int ya = std::max(y0_, static_cast<int>(std::floor(cy - radiusPx)));
			const int yb = std::min(y0_ + h_ - 1, static_cast<int>(std::ceil(cy + radiusPx)));
			const float r2 = radiusPx * radiusPx;
			for (int y = ya; y <= yb; ++y) {
				const float dy = (static_cast<float>(y) + 0.5f) - cy;
				for (int x = xa; x <= xb; ++x) {
					const float dx = (static_cast<float>(x) + 0.5f) - cx;
					if (dx * dx + dy * dy <= r2) write(x, y, d);
				}
			}
		}

		// A triangle in screen space; 1/w is interpolated (perspective-correct distance).
		void triangle(const glm::vec2 s[3], const float w[3])
		{
			const float minX = std::min({ s[0].x, s[1].x, s[2].x }), maxX = std::max({ s[0].x, s[1].x, s[2].x });
			const float minY = std::min({ s[0].y, s[1].y, s[2].y }), maxY = std::max({ s[0].y, s[1].y, s[2].y });
			const int xa = std::max(x0_, static_cast<int>(std::floor(minX))), xb = std::min(x0_ + w_ - 1, static_cast<int>(std::ceil(maxX)));
			const int ya = std::max(y0_, static_cast<int>(std::floor(minY))), yb = std::min(y0_ + h_ - 1, static_cast<int>(std::ceil(maxY)));
			if (xa > xb || ya > yb) return;
			const float area = (s[1].x - s[0].x) * (s[2].y - s[0].y) - (s[2].x - s[0].x) * (s[1].y - s[0].y);
			if (!(std::abs(area) > 1.0e-9f)) return;
			const float invW[3] = { 1.f / w[0], 1.f / w[1], 1.f / w[2] };
			for (int y = ya; y <= yb; ++y) {
				const float py = static_cast<float>(y) + 0.5f;
				for (int x = xa; x <= xb; ++x) {
					const float px = static_cast<float>(x) + 0.5f;
					const float b0 = ((s[1].x - px) * (s[2].y - py) - (s[2].x - px) * (s[1].y - py)) / area;
					const float b1 = ((s[2].x - px) * (s[0].y - py) - (s[0].x - px) * (s[2].y - py)) / area;
					const float b2 = 1.f - b0 - b1;
					if (b0 < 0.f || b1 < 0.f || b2 < 0.f) continue;
					const float inv = b0 * invW[0] + b1 * invW[1] + b2 * invW[2];
					if (inv > 0.f) write(x, y, 1.f / inv);
				}
			}
		}

	private:
		int x0_, y0_, w_, h_;
		std::vector<float> depth_;
	};

	struct Camera
	{
		glm::mat4 viewProj;       // proj * view
		glm::mat4 mvp;            // proj * view * model
		float width, height;
		float modelScale;         // largest stretch the model applies to a unit vector (the sphere radius in world units)
		float pixelsPerUnitAtW1;  // projected size of one world unit at view distance 1
	};

	Camera makeCamera(const PointPickView& v)
	{
		Camera c;
		c.viewProj = v.proj * v.view;
		c.mvp = c.viewProj * v.model;
		c.width = v.width;
		c.height = v.height;
		c.modelScale = std::max({ glm::length(glm::vec3(v.model[0])), glm::length(glm::vec3(v.model[1])), glm::length(glm::vec3(v.model[2])) });
		c.pixelsPerUnitAtW1 = 0.5f * v.height * std::abs(v.proj[1][1]);
		return c;
	}

	struct Projected
	{
		bool ok = false;
		float x = 0.f, y = 0.f;   // pixels
		float w = 0.f;            // view-space distance of the centre
		float radiusPx = 0.f;     // projected sphere radius, not yet clamped
		float frontW = 0.f;       // view-space distance of the sphere's near surface
	};

	Projected project(const Camera& c, const glm::vec3& local, float radius)
	{
		Projected p;
		const glm::vec4 clip = c.mvp * glm::vec4(local, 1.f);
		if (!(clip.w > kNear)) return p;
		const float rWorld = radius * c.modelScale;
		p.ok = true;
		p.w = clip.w;
		p.x = (clip.x / clip.w * 0.5f + 0.5f) * c.width;
		p.y = (clip.y / clip.w * 0.5f + 0.5f) * c.height;
		p.radiusPx = rWorld * c.pixelsPerUnitAtW1 / clip.w;
		p.frontW = clip.w - rWorld;
		return p;
	}

	void addOccluders(DepthMap& map, const Camera& c, const std::vector<OccluderTriangle>& occluders)
	{
		for (const auto& t : occluders) {
			const glm::vec3 v[3] = { t.a, t.b, t.c };
			glm::vec2 s[3];
			float w[3];
			bool ok = true;
			for (int i = 0; i < 3; ++i) {
				const glm::vec4 clip = c.viewProj * glm::vec4(v[i], 1.f);
				if (!(clip.w > kNear)) { ok = false; break; }   // crosses the eye plane: not clipped, simply not an occluder
				w[i] = clip.w;
				s[i] = glm::vec2((clip.x / clip.w * 0.5f + 0.5f) * c.width, (clip.y / clip.w * 0.5f + 0.5f) * c.height);
			}
			if (ok) map.triangle(s, w);
		}
	}

	float tolerance(float w) { return 1.0e-3f * w + 1.0e-6f; }

	float clampRadius(float radiusPx, const PointPickOptions& o)
	{
		return std::min(std::max(radiusPx, o.minRadiusPx), o.maxOccludeRadiusPx);
	}

	// Fills `map` with everything that can hide a point inside it: the occluders, and the discs of all the points of the cloud.
	void buildMap(DepthMap& map, const PointSet& points, const Camera& c, const PointPickOptions& o,
		const std::vector<OccluderTriangle>& occluders, float rx0, float ry0, float rx1, float ry1)
	{
		addOccluders(map, c, occluders);
		for (size_t i = 0; i < points.size(); ++i) {
			const Projected p = project(c, points.positions[i], points.radii[i]);
			if (!p.ok) continue;
			const float r = clampRadius(p.radiusPx, o);
			if (p.x + r < rx0 || p.x - r > rx1 || p.y + r < ry0 || p.y - r > ry1) continue;
			map.splat(p.x, p.y, r, p.frontW);
		}
	}

}

std::optional<uint32_t> Phantom::PC::pickPoint(const PointSet& points, const PointPickView& view, double x, double y,
	const PointPickOptions& options, const std::vector<OccluderTriangle>& occluders)
{
	if (points.empty() || !(view.width > 0.f) || !(view.height > 0.f)) return std::nullopt;
	const Camera c = makeCamera(view);
	const float reach = options.tolerancePx + options.maxOccludeRadiusPx;
	const int ax = std::max(0, static_cast<int>(std::floor(x - reach))), ay = std::max(0, static_cast<int>(std::floor(y - reach)));
	const int bx = std::min(static_cast<int>(view.width), static_cast<int>(std::ceil(x + reach)) + 1);
	const int by = std::min(static_cast<int>(view.height), static_cast<int>(std::ceil(y + reach)) + 1);
	if (bx <= ax || by <= ay) return std::nullopt;

	DepthMap map(ax, ay, bx, by);
	if (!options.xray) buildMap(map, points, c, options, occluders, float(ax), float(ay), float(bx), float(by));

	std::optional<uint32_t> best;
	float bestDistance = std::numeric_limits<float>::infinity(), bestW = std::numeric_limits<float>::infinity();
	for (size_t i = 0; i < points.size(); ++i) {
		const Projected p = project(c, points.positions[i], points.radii[i]);
		if (!p.ok) continue;
		const float r = std::max(p.radiusPx, options.minRadiusPx);
		const float dx = p.x - static_cast<float>(x), dy = p.y - static_cast<float>(y);
		const float distance = std::sqrt(dx * dx + dy * dy);
		if (distance > options.tolerancePx + std::min(r, options.maxOccludeRadiusPx)) continue;
		if (!options.xray && !(p.frontW <= map.at(static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))) + tolerance(p.w))) continue;
		if (distance < bestDistance || (distance == bestDistance && p.w < bestW)) {
			best = static_cast<uint32_t>(i);
			bestDistance = distance;
			bestW = p.w;
		}
	}
	return best;
}

PointIndexList Phantom::PC::boxSelectPoints(const PointSet& points, const PointPickView& view, double x0, double y0, double x1, double y1,
	const PointPickOptions& options, const std::vector<OccluderTriangle>& occluders)
{
	PointIndexList hits;
	if (points.empty() || !(view.width > 0.f) || !(view.height > 0.f)) return hits;
	const float left = static_cast<float>(std::min(x0, x1)), right = static_cast<float>(std::max(x0, x1));
	const float top = static_cast<float>(std::min(y0, y1)), bottom = static_cast<float>(std::max(y0, y1));
	const Camera c = makeCamera(view);

	const int ax = std::max(0, static_cast<int>(std::floor(left))), ay = std::max(0, static_cast<int>(std::floor(top)));
	const int bx = std::min(static_cast<int>(view.width), static_cast<int>(std::ceil(right)) + 1);
	const int by = std::min(static_cast<int>(view.height), static_cast<int>(std::ceil(bottom)) + 1);
	if (bx <= ax || by <= ay) return hits;

	DepthMap map(ax, ay, bx, by);
	if (!options.xray) buildMap(map, points, c, options, occluders, float(ax), float(ay), float(bx), float(by));

	for (size_t i = 0; i < points.size(); ++i) {
		const Projected p = project(c, points.positions[i], points.radii[i]);
		if (!p.ok || p.x < left || p.x > right || p.y < top || p.y > bottom) continue;
		if (!options.xray && !(p.frontW <= map.at(static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))) + tolerance(p.w))) continue;
		hits.push_back(static_cast<uint32_t>(i));
	}
	return hits;
}
