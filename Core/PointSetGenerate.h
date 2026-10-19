#pragma once

#include "PointSet.h"

#include "../../CGLib/Math/Core/glm.h"

#include <cstdint>
#include <vector>

namespace Phantom {
	namespace PC {

		/// A triangle in whatever space the caller works in (PhantomStudio passes world space).
		struct SurfaceTriangle
		{
			glm::vec3 a, b, c;
		};

		/// @brief One point per vertex, all with the same colour (linear RGB) and radius. Normals are left out.
		PointSet pointsFromVertices(const std::vector<Math::Vector3df>& vertices, const Math::Vector3df& color, float radius);

		/// @brief `count` points spread over the triangles with probability proportional to area (a large triangle gets
		/// proportionally more), uniform inside each triangle. Every point carries its triangle's face normal. The result depends only
		/// on (triangles, count, seed): the generator is a fixed SplitMix64, not a library distribution, so it is the same on every
		/// platform and compiler. Degenerate (zero-area) triangles never receive points.
		/// @return an empty set when `count` is 0 or the total area is 0.
		PointSet samplePointsOnSurface(const std::vector<SurfaceTriangle>& triangles, size_t count, uint64_t seed,
			const Math::Vector3df& color, float radius);

		/// Total area of the triangles (double precision).
		double totalArea(const std::vector<SurfaceTriangle>& triangles);

	}
}
