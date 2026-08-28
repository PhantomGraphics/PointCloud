#pragma once

#include "CGLib/Math/Vector3d.h"
#include <cstddef>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Detects boundary points in a point cloud from the angular distribution of
		/// neighbors projected onto the local tangent plane (requires precomputed per-point
		/// normals, e.g. from NormalEstimator). A point is flagged as boundary when its
		/// neighbors leave a wide angular gap around it -- the signature of sitting at the edge
		/// of a surface patch rather than surrounded on all sides.
		class BoundaryDetector
		{
		public:
			BoundaryDetector() = default;
			~BoundaryDetector() = default;

			/// @brief Adds a point with its precomputed normal.
			void add(const Math::Vector3df& position, const Math::Vector3df& normal) {
				positions.push_back(position);
				normals.push_back(normal);
			}

			/// @brief Flags each point as boundary/interior based on the widest angular gap
			/// between neighbors projected onto the tangent plane at that point. Points with
			/// fewer than 3 usable neighbors are left as interior (false): there isn't enough
			/// information to judge.
			/// @param searchRadius The neighborhood search radius.
			/// @param angleThresholdRad Points whose widest angular gap exceeds this are flagged
			///        as boundary. Default ~0.85*pi (153 degrees), following PCL's boundary
			///        estimation default.
			void estimate(const double searchRadius, const float angleThresholdRad = 2.6703537555513248f);

			/// @brief Same as estimate(), but finds each point's neighborhood via its k nearest
			/// neighbors (Space::KDTree) instead of a fixed search radius.
			/// @param k Number of nearest neighbors (excluding the point itself) to use.
			/// @param angleThresholdRad Points whose widest angular gap exceeds this are flagged
			///        as boundary. Default ~0.85*pi (153 degrees), following PCL's boundary
			///        estimation default.
			void estimateKNN(const size_t k = 10, const float angleThresholdRad = 2.6703537555513248f);

			/// @brief Returns the boundary flags, same order as added points.
			std::vector<bool> getBoundaryFlags() const { return boundaryFlags; }

		private:
			std::vector<Math::Vector3df> positions;
			std::vector<Math::Vector3df> normals;
			std::vector<bool> boundaryFlags;
		};

	}
}
