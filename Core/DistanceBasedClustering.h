#pragma once

#include "ProgressReporter.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief A 3D point used for distance-based clustering.
		struct Point {
			double x, y, z;    ///< 3D coordinates.
			int clusterID;     ///< Cluster ID assigned to this point (0: unclassified, 1+: cluster ID).

			/// @brief Constructs a point at the given coordinates (clusterID initialized to 0).
			/// @param _x X coordinate.
			/// @param _y Y coordinate.
			/// @param _z Z coordinate.
			Point(double _x = 0, double _y = 0, double _z = 0)
				: x(_x), y(_y), z(_z), clusterID(0) {
			}
		};

		/// @brief Clusters a point cloud using distance-based region growing.
		class DistanceBasedClustering
		{
		public:
			/// @brief Assigns cluster IDs to points using a distance-based region growing algorithm.
			/// Points within the search radius of a cluster seed are assigned to the same cluster.
			/// @param points The input/output point cloud. Each point's clusterID is updated in place.
			/// @param searchRadius The maximum distance to consider two points as neighbors.
			/// @param reporter Optional progress/cancellation callback (F-3), same throttling and
			///                 cancel-time noise fallback semantics as DBSCANClustering::cluster().
			void DistanceBasedRegionGrowing(std::vector<Point>& points, double searchRadius,
			                                 const ProgressReporter& reporter = {});
		};
	}
}
