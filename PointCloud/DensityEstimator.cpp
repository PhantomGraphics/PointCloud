#include "DensityEstimator.h"

#include "../../CGLib/Math/Gaussian.h"
#include "../../CGLib/Space/Space/CompactSpaceHash.h"
#include "../../CGLib/Space/Space/KDTree.h"

#include <algorithm>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::PC;

void DensityEstimator::estimate(const double searchRadius)
{
	const auto weight = Gaussian::createNormalDistributionFunc(0.0f, 1.0f);

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));

	const auto size = pointCloud.size();
	for (size_t i = 0; i < size; ++i) {
		const auto& position = pointCloud[i];
		spaceHash.add(position);
	}

   this->densities.assign(size, 0.0);

	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto& position = pointCloud[static_cast<size_t>(i)];
		const auto indices = spaceHash.findNeighborIndices(i);
		for (const auto& pi : indices) {
			const auto& p = pointCloud[pi];
			const auto dist = Math::getLength(p - position);
			const auto w = weight.getWeight(static_cast<float>(dist / searchRadius));
			this->densities[static_cast<size_t>(i)] += w;
		}
	}
}

void DensityEstimator::estimateKNN(const size_t k)
{
	const auto weight = Gaussian::createNormalDistributionFunc(0.0f, 1.0f);

	const auto size = pointCloud.size();
	this->densities.assign(size, 0.0);

	if (pointCloud.empty()) {
		return;
	}

	KDTree tree;
	tree.build(pointCloud);

	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto& position = pointCloud[static_cast<size_t>(i)];
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto indices = tree.findKNearestIndices(position, k + 1);
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
		if (indices.size() > k) indices.resize(k);
		if (indices.empty()) {
			continue;
		}

		// Use the farthest of the k neighbors as an adaptive normalization radius (the KNN
		// equivalent of the fixed searchRadius used by estimate()).
		double maxDist = 0.0;
		for (const auto& pi : indices) {
			maxDist = std::max(maxDist, static_cast<double>(Math::getLength(pointCloud[pi] - position)));
		}
		if (maxDist <= 0.0) {
			continue;
		}

		for (const auto& pi : indices) {
			const auto& p = pointCloud[pi];
			const auto dist = Math::getLength(p - position);
			const auto w = weight.getWeight(static_cast<float>(dist / maxDist));
			this->densities[static_cast<size_t>(i)] += w;
		}
	}
}
