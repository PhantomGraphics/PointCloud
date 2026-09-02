#include "DBSCAN.h"

#include <CGLib/Space/Space/CompactSpaceHash.h>

#include <algorithm>
#include <vector>

using namespace Phantom::PC;
using namespace Phantom::Space;
using namespace Phantom::Math;

namespace {
    double distanceSq(const Point& a, const Point& b)
    {
        return (a.x - b.x) * (a.x - b.x)
             + (a.y - b.y) * (a.y - b.y)
             + (a.z - b.z) * (a.z - b.z);
    }
}

void DBSCANClustering::cluster(std::vector<Point>& points, double eps, int minPts,
                                const ProgressReporter& reporter)
{
    const double epsSq = eps * eps;
    int nextClusterID = 1;
    const int n = static_cast<int>(points.size());

    // Build a spatial hash for fast approximate neighbor lookup.
    CompactSpaceHash spaceHash(eps, n);
    for (const auto& p : points) {
        spaceHash.add(Vector3df((float)p.x, (float)p.y, (float)p.z));
    }

    // The spatial hash returns all points in the 3x3x3 cell neighborhood, which
    // slightly exceeds the eps sphere. An exact distance check is required so
    // that the core-point criterion (minPts neighbors within eps) is correct.
    //
    // Deliberately not restructured to precompute-and-cache every point's neighbor list
    // up front (which would allow OpenMP-parallelizing the search): that was tried and
    // measured to be 10-15x SLOWER even single-threaded
    // on dense clustered data, because retaining ~n heap-allocated neighbor-list vectors for
    // the whole run (rather than each findNeighbors() call's result living briefly and being
    // freed right away) blows up peak memory and allocator/cache overhead for workloads where
    // points have many neighbors. Cluster expansion itself is also inherently sequential
    // (each BFS's assignments depend on what earlier BFS runs already claimed), so there's no
    // simple safe parallelization opportunity here without a more substantial redesign (e.g.
    // a flat CSR-style adjacency buffer instead of vector<vector<int>>).
    auto findNeighbors = [&](int i) -> std::vector<int> {
        std::vector<int> result;
        for (int j : spaceHash.findNeighborIndices(i)) {
            if (distanceSq(points[i], points[j]) <= epsSq) {
                result.push_back(j);
            }
        }
        return result;
    };

    // A progress callback crosses the GIL on the Python-bound side, so its per-call
    // cost isn't negligible. Throttle to roughly a constant ~200 calls total regardless
    // of N by only calling report() every reportStride-th outer-loop iteration.
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

        const auto neighbors = findNeighbors(i);
        if ((int)neighbors.size() < minPts) {
            points[i].clusterID = -1; // tentative noise
            continue;
        }

        // Core point: seed a new cluster and expand via BFS.
        points[i].clusterID = nextClusterID;
        std::vector<int> queue = neighbors;
        size_t head = 0;

        while (head < queue.size()) {
            const int cur = queue[head++];
            Point& cp = points[cur];

            if (cp.clusterID > 0) continue; // already assigned to a cluster

            // Assign to this cluster (handles both unclassified and noise points).
            cp.clusterID = nextClusterID;

            // If cur is also a core point, expand the cluster through it.
            const auto curNeighbors = findNeighbors(cur);
            if ((int)curNeighbors.size() >= minPts) {
                for (int j : curNeighbors) {
                    if (points[j].clusterID <= 0) { // unclassified or noise
                        queue.push_back(j);
                    }
                }
            }
        }

        nextClusterID++;
    }

    if (cancelled) {
        // On cancellation, finalize any point left unclassified as noise so the
        // (incomplete) result is still a structurally valid clusterID array
        // (every value is either -1 or a positive cluster id).
        for (auto& p : points) {
            if (p.clusterID == 0) p.clusterID = -1;
        }
    } else {
        reporter.report(1.0f);
    }
}
