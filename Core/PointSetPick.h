#pragma once

#include "PointSet.h"
#include "PointSetEdit.h"

#include "../../CGLib/Math/Core/glm.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Screen-space picking of points (docs/todo/PLAN_phantomstudio_pointcloud.md Phase 3), on the CPU and without a GPU
		/// readback, so a headless session and the unit tests pick exactly like the viewport.
		///
		/// Camera convention: the same as the mesh element picking. `proj` already carries the Vulkan Y flip, so
		/// ndc.y * 0.5 + 0.5 scaled by `height` is the pixel row (origin top-left); `model` maps the cloud's local space to world.
		/// A point is the sphere of its radius under `model` (an ellipsoid under a non-uniform scale).
		/// Coverage and depth use ray/sphere intersection in local space; subpixel points keep a minimum click target.
		struct PointPickView
		{
			glm::mat4 model{ 1.f };
			glm::mat4 view{ 1.f };
			glm::mat4 proj{ 1.f };
			float width = 1.f, height = 1.f;
		};

		struct PointPickOptions
		{
			float radiusScale = 1.f;
			/// false (default): a point hidden behind other points of the cloud or behind an occluder cannot be picked. true: see-through.
			bool xray = false;
			/// How close (pixels) a click must be to a point, beyond the point's own projected radius.
			float tolerancePx = 6.f;
			/// Discs smaller than this are treated as this size, so single-pixel points can still be hit and hide what is behind them.
			float minRadiusPx = 1.5f;
			/// Click-query padding. Occlusion coverage itself is never clamped.
			float maxOccludeRadiusPx = 24.f;
		};

		/// A world-space triangle of a mesh that hides points behind it.
		struct OccluderTriangle
		{
			glm::vec3 a, b, c;
		};

		/// @brief The point under (x, y), or std::nullopt. Among the points within reach the nearest one on screen wins (ties: the
		/// one closer to the camera); points that are not visible are skipped unless `options.xray`.
		std::optional<uint32_t> pickPoint(const PointSet& points, const PointPickView& view, double x, double y,
			const PointPickOptions& options, const std::vector<OccluderTriangle>& occluders);

		/// @brief The points whose centres lie inside the rectangle (corners in any order); sorted, distinct. Hidden points are left
		/// out unless `options.xray`. Points behind the camera are never selected.
		PointIndexList boxSelectPoints(const PointSet& points, const PointPickView& view, double x0, double y0, double x1, double y1,
			const PointPickOptions& options, const std::vector<OccluderTriangle>& occluders);

	}
}
