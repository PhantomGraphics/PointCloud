#pragma once

#include "DistanceBasedClustering.h"
#include "ProgressReporter.h"
#include <vector>

namespace Phantom {
    namespace PC {

        /// @brief Clusters a point cloud using the DBSCAN algorithm.
        ///
        /// After clustering, each point's clusterID is set to one of:
        ///   -1  noise (too few neighbors, not reachable from any core point),
        ///   1+  cluster ID.
        class DBSCANClustering
        {
        public:
            DBSCANClustering() = default;
            ~DBSCANClustering() = default;

            /// @brief Runs DBSCAN on the given point cloud.
            /// @param points Input/output points. clusterID is updated in place.
            /// @param eps    Neighborhood search radius.
            /// @param minPts Minimum neighbor count (including self) required to
            ///               classify a point as a core point.
            /// @param reporter Optional progress/cancellation callback (F-3). Reported at
            ///                 throttled intervals over the outer per-point loop. If
            ///                 cancelled partway through, any point left unclassified
            ///                 (clusterID == 0) is finalized as noise (-1) so the returned
            ///                 labels are still a valid (if incomplete) clustering.
            void cluster(std::vector<Point>& points, double eps, int minPts,
                         const ProgressReporter& reporter = {});
        };

    }
}
