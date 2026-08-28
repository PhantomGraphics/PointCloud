#pragma once

#include "CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Computes the 2D concave ("non-convex") hull of a point cloud's XY projection,
		/// via the k-nearest-neighbours algorithm of Moreira & Santos (2007). This is a distinct,
		/// simpler alternative to alpha shapes (no Delaunay triangulation required): the boundary
		/// is traced by repeatedly hopping from the current hull point to whichever of its k
		/// nearest unused neighbors turns most sharply clockwise without crossing an
		/// already-placed edge, re-trying with a larger k whenever no candidate works or the
		/// resulting polygon fails to contain every input point. As k grows large enough this
		/// degenerates to the ordinary convex hull. z is ignored for the computation; returned
		/// hull points use z=0.
		class ConcaveHull2D
		{
		public:
			ConcaveHull2D() = default;
			~ConcaveHull2D() = default;

			/// @brief Adds a point to be processed (only its x,y are used).
			void add(const Math::Vector3df& position) { pointCloud.push_back(position); }

			/// @brief Computes the concave hull, starting from `k` nearest neighbors per step and
			/// increasing k on failure (see class docs) up to `maxK`.
			/// @param k Initial neighbor count; clamped to at least 3.
			/// @param maxK Upper bound on k during retries; 0 means "as many as needed" (up to
			///        the point count, at which point the result degenerates to the convex hull
			///        and is guaranteed to succeed).
			/// @return true if a valid, simple (non-self-intersecting), all-containing hull was
			///         found. Requires at least 3 distinct points.
			bool compute(size_t k = 3, size_t maxK = 0);

			/// @brief Returns the hull vertices in counter-clockwise order (z=0 for every
			/// vertex); the last vertex is not a repeat of the first. Empty until compute()
			/// succeeds.
			std::vector<Math::Vector3df> getHullPoints() const { return hullPoints; }

			/// @brief Returns the polygon area enclosed by the hull (shoelace formula). 0 until
			/// compute() succeeds.
			float getArea() const { return area; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<Math::Vector3df> hullPoints;
			float area = 0.0f;
		};

	}
}
