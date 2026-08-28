#include "RegionGrowing.h"
#include "../../CGLib/Space/Space/KDTree.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::PC;

bool RegionGrowing::segment(const Params& params)
{
	labels.clear();
	clusterCount = 0;

	const size_t n = positions.size();
	if (n == 0 || normals.size() != n || curvatures.size() != n) {
		return false;
	}

	KDTree tree;
	tree.build(positions);

	// Grow from the smoothest (lowest-curvature) points first, as their neighborhood normal is
	// the most reliable seed for a planar/smooth patch.
	std::vector<size_t> order(n);
	std::iota(order.begin(), order.end(), size_t(0));
	std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
		return curvatures[a] < curvatures[b];
	});

	const float cosThreshold = std::cos(params.smoothnessThresholdRad);

	labels.assign(n, -1);
	std::vector<bool> visited(n, false);

	for (const size_t seed : order) {
		if (visited[seed]) continue;

		std::vector<size_t> region;
		std::deque<size_t> queue;
		visited[seed] = true;
		region.push_back(seed);
		queue.push_back(seed);

		while (!queue.empty()) {
			const size_t cur = queue.front();
			queue.pop_front();

			const auto neighborIndices = tree.findKNearestIndices(positions[cur], params.kNeighbors);
			for (const int ni : neighborIndices) {
				const size_t neighbor = static_cast<size_t>(ni);
				if (visited[neighbor]) continue;

				// abs() so an arbitrarily-flipped normal sign (PCA normals aren't oriented by
				// default -- see NormalEstimator::orientTowardsViewpoint()) doesn't spuriously
				// block growth across an otherwise-smooth surface.
				const float cosAngle = std::fabs(glm::dot(normals[cur], normals[neighbor]));
				if (cosAngle < cosThreshold) continue;

				visited[neighbor] = true;
				region.push_back(neighbor);
				if (curvatures[neighbor] < params.curvatureThreshold) {
					queue.push_back(neighbor);
				}
			}
		}

		if (region.size() >= params.minClusterSize) {
			for (const size_t idx : region) {
				labels[idx] = static_cast<int>(clusterCount);
			}
			++clusterCount;
		}
		// Otherwise: leave as unclustered (-1). Points stay `visited` either way, so a discarded
		// small region is never re-examined as a seed for a different cluster.
	}

	return true;
}
