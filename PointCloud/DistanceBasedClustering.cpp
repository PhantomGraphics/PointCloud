#include "DistanceBasedClustering.h"

#include <CGLib/Space/Space/CompactSpaceHash.h>

#include <algorithm>
#include <queue>

using namespace Phantom::PC;
using namespace Phantom::Space;
using namespace Phantom::Math;

void DistanceBasedClustering::DistanceBasedRegionGrowing(std::vector<Point>& points, double searchRadius,
                                                           const ProgressReporter& reporter)
{
    int nextClusterID = 1;
    const int n = static_cast<int>(points.size());

    // Build a spatial hash for O(1) average-case neighbor lookup,
    // replacing the previous O(n^2) linear scan.
    CompactSpaceHash spaceHash(searchRadius, n);
    for (const auto& p : points) {
        spaceHash.add(Vector3df((float)p.x, (float)p.y, (float)p.z));
    }

    // Throttled for the same reason as DBSCANClustering::cluster() (F-3).
    const int reportStride = reporter.isSet() ? std::max(1, n / 200) : 0;
    bool cancelled = false;

    for (int i = 0; i < n; ++i) {
        if (reportStride != 0 && (i % reportStride) == 0) {
            if (!reporter.report(static_cast<float>(i) / static_cast<float>(n))) {
                cancelled = true;
                break;
            }
        }

        if (points[i].clusterID != 0) continue;

        // Seed a new cluster from this unclassified point.
        points[i].clusterID = nextClusterID;
        std::queue<int> growQueue;
        growQueue.push(i);

        // BFS region growing: expand into spatially adjacent unclassified points.
        while (!growQueue.empty()) {
            const int current = growQueue.front();
            growQueue.pop();

            for (int j : spaceHash.findNeighborIndices(current)) {
                if (points[j].clusterID != 0) continue;
                points[j].clusterID = nextClusterID;
                growQueue.push(j);
            }
        }

        nextClusterID++;
    }

    if (cancelled) {
        // On cancellation, mark any point left unclassified with -1, same as
        // DBSCANClustering::cluster() -- a value this algorithm otherwise never
        // produces, so it unambiguously means "never reached".
        for (auto& p : points) {
            if (p.clusterID == 0) p.clusterID = -1;
        }
    } else {
        reporter.report(1.0f);
    }
}
