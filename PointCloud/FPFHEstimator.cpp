#include "FPFHEstimator.h"
#include "../../CGLib/Space/Space/KDTree.h"

#include <algorithm>
#include <cmath>
#include <utility>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::PC;

namespace {
	constexpr float kPi = 3.14159265358979323846f;
}

bool FPFHEstimator::computePairFeatures(
	const Vector3df& sourcePosition, const Vector3df& sourceNormal,
	const Vector3df& targetPosition, const Vector3df& targetNormal,
	float& outAlpha, float& outPhi, float& outTheta)
{
	const Vector3df d = targetPosition - sourcePosition;
	const float dist = glm::length(d);
	if (dist < 1.0e-12f) {
		return false;
	}
	const Vector3df dn = d / dist;

	const Vector3df u = sourceNormal;
	Vector3df v = glm::cross(dn, u);
	const float vLen = glm::length(v);
	if (vLen < 1.0e-6f) {
		// d is (nearly) parallel to the source normal: the Darboux frame is degenerate.
		return false;
	}
	v /= vLen;
	const Vector3df w = glm::cross(u, v);

	outAlpha = glm::dot(v, targetNormal);
	outPhi = glm::dot(u, dn);
	outTheta = std::atan2(glm::dot(w, targetNormal), glm::dot(u, targetNormal));
	return true;
}

FPFHEstimator::Histogram FPFHEstimator::computeSPFH(
	size_t index,
	const std::vector<int>& neighborIndices,
	const std::vector<Vector3df>& positions,
	const std::vector<Vector3df>& normals)
{
	Histogram hist{};
	hist.fill(0.0f);
	if (neighborIndices.empty()) {
		return hist;
	}

	const auto& position = positions[index];
	const auto& normal = normals[index];

	int counted = 0;
	for (const int ni : neighborIndices) {
		if (ni == static_cast<int>(index)) continue;
		float alpha = 0.0f, phi = 0.0f, theta = 0.0f;
		if (!computePairFeatures(position, normal, positions[ni], normals[ni], alpha, phi, theta)) {
			continue;
		}
		// alpha, phi in [-1, 1]; theta in [-pi/2, pi/2].
		const int alphaBin = std::clamp(static_cast<int>((alpha + 1.0f) * 0.5f * BinsPerFeature), 0, BinsPerFeature - 1);
		const int phiBin = std::clamp(static_cast<int>((phi + 1.0f) * 0.5f * BinsPerFeature), 0, BinsPerFeature - 1);
		const int thetaBin = std::clamp(static_cast<int>((theta + kPi * 0.5f) / kPi * BinsPerFeature), 0, BinsPerFeature - 1);

		hist[alphaBin] += 1.0f;
		hist[BinsPerFeature + phiBin] += 1.0f;
		hist[2 * BinsPerFeature + thetaBin] += 1.0f;
		++counted;
	}

	if (counted > 0) {
		// Normalize each of the 3 sub-histograms independently so the histogram shape doesn't
		// depend on local point density.
		for (int feature = 0; feature < 3; ++feature) {
			float sum = 0.0f;
			for (int b = 0; b < BinsPerFeature; ++b) sum += hist[feature * BinsPerFeature + b];
			if (sum > 0.0f) {
				for (int b = 0; b < BinsPerFeature; ++b) hist[feature * BinsPerFeature + b] /= sum;
			}
		}
	}
	return hist;
}

bool FPFHEstimator::estimate(size_t k)
{
	histograms.clear();

	if (positions.size() < 2 || k < 1) {
		return false;
	}

	KDTree tree;
	tree.build(positions);

	const size_t n = positions.size();
	std::vector<std::vector<int>> neighborLists(n);

	const int nInt = static_cast<int>(n);
#pragma omp parallel for
	for (int i = 0; i < nInt; ++i) {
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto neighbors = tree.findKNearestIndices(positions[static_cast<size_t>(i)], k + 1);
		neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), i), neighbors.end());
		if (neighbors.size() > k) neighbors.resize(k);
		neighborLists[static_cast<size_t>(i)] = std::move(neighbors);
	}

	std::vector<Histogram> spfh(n);
#pragma omp parallel for
	for (int i = 0; i < nInt; ++i) {
		spfh[static_cast<size_t>(i)] = computeSPFH(static_cast<size_t>(i), neighborLists[static_cast<size_t>(i)], positions, normals);
	}

	histograms.resize(n);
#pragma omp parallel for
	for (int i = 0; i < nInt; ++i) {
		Histogram combined = spfh[static_cast<size_t>(i)];
		const auto& neighbors = neighborLists[static_cast<size_t>(i)];
		if (!neighbors.empty()) {
			Histogram weightedSum{};
			weightedSum.fill(0.0f);
			for (const int ni : neighbors) {
				const float dist = glm::length(positions[ni] - positions[i]);
				if (dist < 1.0e-12f) continue;
				const float weight = 1.0f / dist;
				for (int b = 0; b < HistogramSize; ++b) {
					weightedSum[b] += weight * spfh[ni][b];
				}
			}
			const float invK = 1.0f / static_cast<float>(neighbors.size());
			for (int b = 0; b < HistogramSize; ++b) {
				combined[b] += weightedSum[b] * invK;
			}
		}
		histograms[static_cast<size_t>(i)] = combined;
	}

	return true;
}
