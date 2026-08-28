#include "BoundaryDetector.h"
#include "../../CGLib/Space/Space/CompactSpaceHash.h"
#include "../../CGLib/Space/Space/KDTree.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::PC;

namespace {
	constexpr float kTwoPi = 6.283185307179586f;

	// Boundary flag at positions[i] from its neighbor indices (self excluded) and precomputed
	// normal, shared by BoundaryDetector::estimate() and estimateKNN(). Returns false (interior)
	// if there are fewer than 3 usable neighbors.
	bool isBoundaryPoint(const std::vector<Vector3df>& positions, const std::vector<Vector3df>& normals, size_t i, const std::vector<int>& indices, const float angleThresholdRad) {
		if (indices.size() < 3) {
			return false; // Not enough neighbors to judge -> leave as interior.
		}

		const auto& position = positions[i];
		const auto& normal = normals[i];

		// Build an orthonormal tangent-plane basis (u, v) perpendicular to normal.
		Vector3df helperAxis(1.0f, 0.0f, 0.0f);
		if (std::abs(normal.x) > 0.9f) {
			helperAxis = Vector3df(0.0f, 1.0f, 0.0f);
		}
		const Vector3df u = glm::normalize(glm::cross(normal, helperAxis));
		const Vector3df v = glm::cross(normal, u);

		std::vector<float> angles;
		angles.reserve(indices.size());
		for (const auto ni : indices) {
			const Vector3df d = positions[ni] - position;
			const float pu = glm::dot(d, u);
			const float pv = glm::dot(d, v);
			if (pu == 0.0f && pv == 0.0f) {
				continue; // neighbor projects exactly onto the query point itself
			}
			angles.push_back(std::atan2(pv, pu));
		}
		if (angles.size() < 3) {
			return false;
		}

		std::sort(angles.begin(), angles.end());

		float maxGap = (angles.front() + kTwoPi) - angles.back(); // wraparound gap
		for (size_t k = 1; k < angles.size(); ++k) {
			maxGap = std::max(maxGap, angles[k] - angles[k - 1]);
		}

		return maxGap > angleThresholdRad;
	}
}

void BoundaryDetector::estimate(const double searchRadius, const float angleThresholdRad)
{
	boundaryFlags.assign(positions.size(), false);

	if (positions.empty()) {
		return;
	}

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(positions.size()));
	for (const auto& p : positions) spaceHash.add(p);

	const int n = static_cast<int>(positions.size());
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto indices = spaceHash.findNeighborIndices(i);
		boundaryFlags[static_cast<size_t>(i)] = ::isBoundaryPoint(positions, normals, static_cast<size_t>(i), indices, angleThresholdRad);
	}
}

void BoundaryDetector::estimateKNN(const size_t k, const float angleThresholdRad)
{
	boundaryFlags.assign(positions.size(), false);

	if (positions.empty()) {
		return;
	}

	KDTree tree;
	tree.build(positions);

	const int n = static_cast<int>(positions.size());
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto indices = tree.findKNearestIndices(positions[static_cast<size_t>(i)], k + 1);
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
		if (indices.size() > k) indices.resize(k);

		boundaryFlags[static_cast<size_t>(i)] = ::isBoundaryPoint(positions, normals, static_cast<size_t>(i), indices, angleThresholdRad);
	}
}
