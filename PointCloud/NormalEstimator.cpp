#include "NormalEstimator.h"

#include "../../CGLib/Numerics/Numerics/SVD3d.h"
#include "../../CGLib/Space/Space/CompactSpaceHash.h"
#include "../../CGLib/Space/Space/KDTree.h"
#include <algorithm>

using namespace Phantom::Math;
using namespace Phantom::Numerics;
using namespace Phantom::Space;
using namespace Phantom::PC;

namespace {
	// Covariance is symmetric, so only the 6 upper-triangular entries need accumulating (the
	// original computed all 9 as independent passes over the points -- 1.5x the necessary work).
	// Centered coordinates are also computed once per point instead of once per (i,j) pair.
	Matrix3dd calculateCovarianceMatrix(const std::vector<Vector3df>& points) {
		Vector3dd means(0, 0, 0);
		for (const auto& p : points) {
			means += Vector3dd(p);
		}
		means /= static_cast<double>(points.size());

		double c00 = 0.0, c01 = 0.0, c02 = 0.0, c11 = 0.0, c12 = 0.0, c22 = 0.0;
		for (const auto& p : points) {
			const double dx = means.x - p.x;
			const double dy = means.y - p.y;
			const double dz = means.z - p.z;
			c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
			c11 += dy * dy; c12 += dy * dz;
			c22 += dz * dz;
		}
		const double invN = 1.0 / static_cast<double>(points.size() - 1);

		Matrix3dd matrix(0.0);
		matrix[0][0] = c00 * invN; matrix[0][1] = c01 * invN; matrix[0][2] = c02 * invN;
		matrix[1][0] = c01 * invN; matrix[1][1] = c11 * invN; matrix[1][2] = c12 * invN;
		matrix[2][0] = c02 * invN; matrix[2][1] = c12 * invN; matrix[2][2] = c22 * invN;
		return matrix;
	}

	// Fits the PCA normal at pointCloud[i] from its neighbor indices (self excluded), following
	// the same convention as NormalEstimator::estimate(): the query point itself is included as
	// a zero offset so the covariance matrix is anchored there, and at least 2 actual neighbors
	// are required. `scratch` is caller-owned and reused across calls (cleared here) so the
	// per-point neighbor list doesn't heap-allocate a fresh std::vector every call.
	Vector3df fitNormal(const std::vector<Vector3df>& pointCloud, size_t i, const std::vector<int>& indices,
		std::vector<Vector3df>& scratch) {
		const auto& position = pointCloud[i];
		scratch.clear();
		scratch.emplace_back(0.0f, 0.0f, 0.0f);
		for (const auto& pi : indices) {
			scratch.push_back(pointCloud[pi] - position);
		}
		if (scratch.size() < 3) {
			return Vector3df(0.0f, 0.0f, 0.0f);
		}
		const auto matrix = ::calculateCovarianceMatrix(scratch);
		SVD3d svd;
		const auto res = svd.calculate(matrix);
		const auto col = res.eigenVectors[0];
		return Vector3df(static_cast<float>(col[0]), static_cast<float>(col[1]), static_cast<float>(col[2]));
	}
}

void NormalEstimator::estimate(const double searchRadius)
{
	normals.clear();

	if (pointCloud.empty()) {
		return;
	}

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));

	const auto size = pointCloud.size();
	for (size_t i = 0; i < size; ++i) {
		const auto& position = pointCloud[i];
		spaceHash.add(position);
	}

	normals.resize(size);
	const int n = static_cast<int>(size);
	// spaceHash.findNeighborIndices() only reads the already-built hash table, so concurrent
	// queries below are safe; `scratch` is per-thread since it's declared inside the parallel
	// region (see NormalEstimator.h/fitNormal doc: reused across calls to avoid reallocating).
#pragma omp parallel
	{
		std::vector<Vector3df> scratch;
#pragma omp for
		for (int i = 0; i < n; ++i) {
			const auto indices = spaceHash.findNeighborIndices(i);
			normals[static_cast<size_t>(i)] = ::fitNormal(pointCloud, static_cast<size_t>(i), indices, scratch);
		}
	}
}

void NormalEstimator::estimateKNN(const size_t k)
{
	normals.clear();

	if (pointCloud.empty()) {
		return;
	}

	KDTree tree;
	tree.build(pointCloud);

	const auto size = pointCloud.size();
	normals.resize(size);
	const int n = static_cast<int>(size);
#pragma omp parallel
	{
		std::vector<Vector3df> scratch;
#pragma omp for
		for (int i = 0; i < n; ++i) {
			// +1 since the query point itself is typically returned as its own nearest neighbor.
			auto indices = tree.findKNearestIndices(pointCloud[static_cast<size_t>(i)], k + 1);
			indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
			if (indices.size() > k) indices.resize(k);

			normals[static_cast<size_t>(i)] = ::fitNormal(pointCloud, static_cast<size_t>(i), indices, scratch);
		}
	}
}

void NormalEstimator::orientTowardsViewpoint(const Vector3df& viewpoint)
{
	const auto size = std::min(pointCloud.size(), normals.size());
	for (size_t i = 0; i < size; ++i) {
		const auto toViewpoint = viewpoint - pointCloud[i];
		if (glm::dot(normals[i], toViewpoint) < 0.0f) {
			normals[i] = -normals[i];
		}
	}
}
