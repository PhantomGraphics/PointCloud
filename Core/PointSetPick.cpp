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
		glm::mat4 inverseMvp;
		float width, height;
		glm::vec3 clipRadius;     // local sphere's extent in clip x, y and w, per unit radius
	};

	Camera makeCamera(const PointPickView& v)
	{
		Camera c;
		c.viewProj = v.proj * v.view;
		c.mvp = c.viewProj * v.model;
		c.inverseMvp = glm::inverse(c.mvp);
		c.width = v.width;
		c.height = v.height;
		auto rowLength = [&](int row) { return glm::length(glm::vec3(c.mvp[0][row], c.mvp[1][row], c.mvp[2][row])); };
		c.clipRadius = { rowLength(0), rowLength(1), rowLength(3) };
		return c;
	}

	struct Projected
	{
		bool ok = false;
		float x = 0.f, y = 0.f;   // pixels
		float w = 0.f;            // view-space distance of the centre
		float radiusPx = 0.f;     // projected sphere radius, not yet clamped
	};

	Projected project(const Camera& c, const glm::vec3& local, float radius)
	{
		Projected p;
		const glm::vec4 clip = c.mvp * glm::vec4(local, 1.f);
		if (!(clip.w > kNear)) return p;
		p.ok = true;
		p.w = clip.w;
		p.x = (clip.x / clip.w * 0.5f + 0.5f) * c.width;
		p.y = (clip.y / clip.w * 0.5f + 0.5f) * c.height;
		// Conservative bound only. Actual coverage and depth come from local ray/sphere intersection.
		const float denominator = std::max(kNear, clip.w - radius * c.clipRadius.z);
		const float rx = 0.5f * c.width * radius * (c.clipRadius.x + std::abs(clip.x / clip.w) * c.clipRadius.z) / denominator;
		const float ry = 0.5f * c.height * radius * (c.clipRadius.y + std::abs(clip.y / clip.w) * c.clipRadius.z) / denominator;
		p.radiusPx = std::max(rx, ry);
		return p;
	}

	float surfaceDepth(const Camera& c, const glm::vec3& centre, float radius, float x, float y)
	{
		const float inf = std::numeric_limits<float>::infinity();
		if (!(radius > 0.f)) return inf;
		const glm::vec2 ndc(2.f * x / c.width - 1.f, 2.f * y / c.height - 1.f);
		const glm::vec4 a = c.inverseMvp * glm::vec4(ndc, 0.f, 1.f);
		const glm::vec4 b = c.inverseMvp * glm::vec4(ndc, 0.5f, 1.f);
		if (a.w == 0.f || b.w == 0.f) return inf;
		const glm::vec3 origin = glm::vec3(a) / a.w;
		const glm::dvec3 direction = glm::dvec3(glm::vec3(b) / b.w) - glm::dvec3(origin);
		const glm::dvec3 offset = glm::dvec3(origin) - glm::dvec3(centre);
		const double aa = glm::dot(direction, direction), halfB = glm::dot(offset, direction);
		const double cc = glm::dot(offset, offset) - double(radius) * radius;
		const double discriminant = halfB * halfB - aa * cc;
		if (!(aa > 0.0) || discriminant < 0.0) return inf;
		double t = (-halfB - std::sqrt(discriminant)) / aa;
		if (t < 0.0) t = (-halfB + std::sqrt(discriminant)) / aa;
		if (t < 0.0) return inf;
		const glm::vec4 clip = c.mvp * glm::vec4(glm::vec3(glm::dvec3(origin) + t * direction), 1.f);
		return clip.w > kNear && clip.z >= 0.f && clip.z <= clip.w ? clip.w : inf;
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

	// Fills `map` with everything that can hide a point inside it: the occluders, and the discs of all the points of the cloud.
	void buildMap(DepthMap& map, const PointSet& points, const Camera& c, const PointPickOptions& o,
		const std::vector<OccluderTriangle>& occluders, float rx0, float ry0, float rx1, float ry1)
	{
		addOccluders(map, c, occluders);
		for (size_t i = 0; i < points.size(); ++i) {
			const float radius = points.radii[i] * o.radiusScale;
			const Projected p = project(c, points.positions[i], radius);
			if (!p.ok) continue;
			const float r = std::max(p.radiusPx, o.minRadiusPx);
			if (p.x + r < rx0 || p.x - r > rx1 || p.y + r < ry0 || p.y - r > ry1) continue;
			const int xa = int(std::max(rx0, std::floor(p.x - r))), xb = int(std::min(rx1 - 1.f, std::ceil(p.x + r)));
			const int ya = int(std::max(ry0, std::floor(p.y - r))), yb = int(std::min(ry1 - 1.f, std::ceil(p.y + r)));
			for (int y = ya; y <= yb; ++y) for (int x = xa; x <= xb; ++x) {
				float depth = surfaceDepth(c, points.positions[i], radius, float(x) + 0.5f, float(y) + 0.5f);
				// A subpixel point remains selectable, without enlarging larger ellipsoids.
				if (!std::isfinite(depth) && p.radiusPx < o.minRadiusPx &&
					glm::length(glm::vec2(float(x) + 0.5f - p.x, float(y) + 0.5f - p.y)) <= o.minRadiusPx) depth = p.w;
				map.write(x, y, depth);
			}
		}
	}

}

std::optional<uint32_t> Phantom::PC::pickPoint(const PointSet& points, const PointPickView& view, double x, double y,
	const PointPickOptions& options, const std::vector<OccluderTriangle>& occluders)
{
	if (points.empty() || !(view.width > 0.f) || !(view.height > 0.f) || !(options.radiusScale > 0.f) || glm::determinant(view.model) == 0.f) return std::nullopt;
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
		const float radius = points.radii[i] * options.radiusScale;
		const Projected p = project(c, points.positions[i], radius);
		if (!p.ok) continue;
		const float r = std::max(p.radiusPx, options.minRadiusPx);
		const float dx = p.x - static_cast<float>(x), dy = p.y - static_cast<float>(y);
		const float distance = std::sqrt(dx * dx + dy * dy);
		if (distance > options.tolerancePx + r) continue;
		bool covered = false;
		const int reachPx = static_cast<int>(std::ceil(options.tolerancePx));
		for (int py = std::max(ay, int(std::floor(y)) - reachPx); py < by && py <= int(std::floor(y)) + reachPx && !covered; ++py)
			for (int px = std::max(ax, int(std::floor(x)) - reachPx); px < bx && px <= int(std::floor(x)) + reachPx; ++px) {
				if (glm::length(glm::vec2(float(px) + 0.5f - float(x), float(py) + 0.5f - float(y))) > options.tolerancePx + 0.75f) continue;
				float depth = surfaceDepth(c, points.positions[i], radius, float(px) + 0.5f, float(py) + 0.5f);
				if (!std::isfinite(depth) && p.radiusPx < options.minRadiusPx &&
					glm::length(glm::vec2(float(px) + 0.5f - p.x, float(py) + 0.5f - p.y)) <= options.minRadiusPx) depth = p.w;
				if (std::isfinite(depth) && (options.xray || depth <= map.at(px, py) + tolerance(depth))) { covered = true; break; }
			}
		if (!covered) continue;
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
	if (points.empty() || !(view.width > 0.f) || !(view.height > 0.f) || !(options.radiusScale > 0.f) || glm::determinant(view.model) == 0.f) return hits;
	const float left = static_cast<float>(std::min(x0, x1)), right = static_cast<float>(std::max(x0, x1));
	const float top = static_cast<float>(std::min(y0, y1)), bottom = static_cast<float>(std::max(y0, y1));
	const Camera c = makeCamera(view);

	const int ax = std::max(0, static_cast<int>(std::floor(left))), ay = std::max(0, static_cast<int>(std::floor(top)));
	const int bx = std::min(static_cast<int>(view.width), static_cast<int>(std::ceil(right)) + 1);
	const int by = std::min(static_cast<int>(view.height), static_cast<int>(std::ceil(bottom)) + 1);
	if (bx <= ax || by <= ay) return hits;
	// X-ray rectangles only test centres: no sphere intersection or depth buffer is needed.
	if (options.xray) {
		for (size_t i = 0; i < points.size(); ++i) {
			const glm::vec4 clip = c.mvp * glm::vec4(points.positions[i], 1.f);
			if (!(clip.w > kNear)) continue;
			const float px = (clip.x / clip.w * 0.5f + 0.5f) * c.width;
			const float py = (clip.y / clip.w * 0.5f + 0.5f) * c.height;
			if (px >= left && px <= right && py >= top && py <= bottom) hits.push_back(static_cast<uint32_t>(i));
		}
		return hits;
	}

	DepthMap map(ax, ay, bx, by);
	if (!options.xray) buildMap(map, points, c, options, occluders, float(ax), float(ay), float(bx), float(by));

	for (size_t i = 0; i < points.size(); ++i) {
		const float radius = points.radii[i] * options.radiusScale;
		const Projected p = project(c, points.positions[i], radius);
		if (!p.ok || p.x < left || p.x > right || p.y < top || p.y > bottom) continue;
		const int px = static_cast<int>(std::floor(p.x)), py = static_cast<int>(std::floor(p.y));
		float depth = surfaceDepth(c, points.positions[i], radius, float(px) + 0.5f, float(py) + 0.5f);
		if (!std::isfinite(depth) && p.radiusPx < options.minRadiusPx) depth = p.w;
		if (!options.xray && !(std::isfinite(depth) && depth <= map.at(px, py) + tolerance(depth))) continue;
		hits.push_back(static_cast<uint32_t>(i));
	}
	return hits;
}
