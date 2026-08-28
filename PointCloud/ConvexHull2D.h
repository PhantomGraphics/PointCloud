#pragma once

#include "CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Computes the 2D convex hull of a point cloud's XY projection, via Andrew's
		/// monotone chain algorithm (O(n log n)). Useful for footprint/cross-section area and
		/// boundary extraction of scanned data. z is ignored entirely for the computation, and
		/// returned hull points use z=0 -- callers who need the original z back must look it up
		/// by (x,y) themselves.
		class ConvexHull2D
		{
		public:
			ConvexHull2D() = default;
			~ConvexHull2D() = default;

			/// @brief Adds a point to be processed (only its x,y are used).
			void add(const Math::Vector3df& position) { pointCloud.push_back(position); }

			/// @brief Computes the convex hull of the added points' XY projection. Requires at
			/// least 3 non-collinear points; otherwise the hull is left empty and this returns
			/// false.
			/// @return true if a valid (>=3 vertex) hull was computed.
			bool compute();

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
