#include "MLSSurface.h"
#include "../../CGLib/Space/Space/CompactSpaceHash.h"
#include "../../CGLib/Space/Space/KDTree.h"
#include <CGLib/Math/Matrix3d.h>
#include <CGLib/Numerics/Numerics/SVD3d.h>

#include <algorithm>
#include <cmath>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Numerics;
using namespace Phantom::PC;

namespace {
	// Double-precision covariance matrix, anchored at the query point (one of the input vectors
	// is always the zero offset -- see callers), matching the convention used by
	// NormalEstimator/CurvatureEstimator so the smallest-eigenvalue eigenvector is the surface
	// normal and the other two span an orthonormal tangent basis.
	Matrix3dd calculateCovarianceMatrixDouble(const std::vector<Vector3df>& points) {
		Vector3dd means(0, 0, 0);
		for (const auto& p : points) {
			means += Vector3dd(p);
		}
		means /= static_cast<double>(points.size());

		Matrix3dd matrix(0.0);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < (int)points.size(); k++)
					matrix[i][j] += (means[i] - points[k][i]) *
					(means[j] - points[k][j]);
				matrix[i][j] /= points.size() - 1;
			}
		}
		return matrix;
	}

	struct LocalFrame {
		Vector3dd normal{ 0.0, 0.0, 0.0 };
		Vector3dd tangentU{ 0.0, 0.0, 0.0 };
		Vector3dd tangentV{ 0.0, 0.0, 0.0 };
	};

	// vectors must already include the query point itself (as the zero offset); requires at
	// least 2 further real neighbors (3 vectors total) to define a rank-2 covariance matrix.
	bool buildLocalFrame(const std::vector<Vector3df>& vectors, LocalFrame& frame) {
		if (vectors.size() < 3) {
			return false;
		}
		const auto covariance = ::calculateCovarianceMatrixDouble(vectors);
		SVD3d svd;
		const auto res = svd.calculate(covariance);
		frame.normal = res.eigenVectors[0];
		frame.tangentU = res.eigenVectors[1];
		frame.tangentV = res.eigenVectors[2];
		return true;
	}

	// Generic NxN Gauss-Jordan elimination with partial pivoting for the weighted normal
	// equations below (N is 3 for a plane fit, 6 for a quadric fit).
	bool solveLinearSystem(std::vector<std::vector<double>> A, std::vector<double> b, std::vector<double>& outX) {
		const size_t n = A.size();
		for (size_t col = 0; col < n; ++col) {
			size_t pivotRow = col;
			double pivotMag = std::abs(A[col][col]);
			for (size_t row = col + 1; row < n; ++row) {
				const double mag = std::abs(A[row][col]);
				if (mag > pivotMag) {
					pivotMag = mag;
					pivotRow = row;
				}
			}
			if (pivotMag < 1.0e-12) {
				return false;
			}
			if (pivotRow != col) {
				std::swap(A[col], A[pivotRow]);
				std::swap(b[col], b[pivotRow]);
			}
			const double pivot = A[col][col];
			for (size_t j = 0; j < n; ++j) A[col][j] /= pivot;
			b[col] /= pivot;
			for (size_t row = 0; row < n; ++row) {
				if (row == col) continue;
				const double factor = A[row][col];
				if (factor == 0.0) continue;
				for (size_t j = 0; j < n; ++j) A[row][j] -= factor * A[col][j];
				b[row] -= factor * b[col];
			}
		}
		outX = b;
		return true;
	}

	// Fits a weighted height field h(u,v) in the (tangentU, tangentV, normal) frame from
	// `vectors` (offsets relative to the query point, including the zero offset for the query
	// point itself). order=2 fits a quadric (a,b,c,d,e,f for a*u^2+b*u*v+c*v^2+d*u+e*v+f);
	// order=1 fits a plane (d,e,f for d*u+e*v+f). Weight is a Gaussian kernel on tangent-plane
	// distance with standard deviation `sigma`. Returns false if the (weighted) system is
	// singular or there are fewer vectors than unknowns.
	bool fitHeightField(const std::vector<Vector3df>& vectors, const LocalFrame& frame,
		const double sigma, const int order, std::vector<double>& outCoeffs) {
		const size_t numUnknowns = (order == 2) ? 6 : 3;
		if (vectors.size() < numUnknowns) {
			return false;
		}

		std::vector<std::vector<double>> A(numUnknowns, std::vector<double>(numUnknowns, 0.0));
		std::vector<double> b(numUnknowns, 0.0);

		for (const auto& v : vectors) {
			const Vector3dd vd(v);
			const double u = glm::dot(vd, frame.tangentU);
			const double w = glm::dot(vd, frame.tangentV);
			const double h = glm::dot(vd, frame.normal);
			const double distSq = u * u + w * w;
			const double weight = std::exp(-distSq / (2.0 * sigma * sigma));

			std::vector<double> phi;
			if (order == 2) {
				phi = { u * u, u * w, w * w, u, w, 1.0 };
			} else {
				phi = { u, w, 1.0 };
			}

			for (size_t r = 0; r < numUnknowns; ++r) {
				for (size_t c = 0; c < numUnknowns; ++c) {
					A[r][c] += weight * phi[r] * phi[c];
				}
				b[r] += weight * phi[r] * h;
			}
		}
		return ::solveLinearSystem(A, b, outCoeffs);
	}

	double evaluateHeightField(const std::vector<double>& coeffs, const int order, const double u, const double w) {
		if (order == 2) {
			return coeffs[0] * u * u + coeffs[1] * u * w + coeffs[2] * w * w + coeffs[3] * u + coeffs[4] * w + coeffs[5];
		}
		return coeffs[0] * u + coeffs[1] * w + coeffs[2];
	}

	struct SurfaceFit {
		bool valid = false;
		LocalFrame frame;
		std::vector<double> coeffs;
		int order = 1;
	};

	// Builds the local tangent frame and fits the weighted height field at pointCloud[i] from
	// its neighbor indices (self excluded), shared by MLSSurface::smooth()/smoothKNN() and
	// upsample()/upsampleKNN().
	SurfaceFit fitSurface(const std::vector<Vector3df>& pointCloud, size_t i, const std::vector<int>& indices, double sigma) {
		SurfaceFit fit;
		const auto& position = pointCloud[i];

		std::vector<Vector3df> vectors;
		vectors.emplace_back(0.0f, 0.0f, 0.0f);
		for (const auto ni : indices) {
			vectors.push_back(pointCloud[ni] - position);
		}

		if (!buildLocalFrame(vectors, fit.frame)) {
			return fit;
		}

		fit.order = (vectors.size() >= 6) ? 2 : 1;
		fit.valid = fitHeightField(vectors, fit.frame, sigma, fit.order, fit.coeffs);
		return fit;
	}

	// The KNN equivalent of searchRadius/2 (the fixed Gaussian sigma used by the radius-based
	// smooth()/upsample()): half the distance to the farthest of the given neighbor indices.
	double adaptiveSigma(const std::vector<Vector3df>& pointCloud, size_t i, const std::vector<int>& indices) {
		double maxDist = 0.0;
		const auto& position = pointCloud[i];
		for (const auto ni : indices) {
			maxDist = std::max(maxDist, static_cast<double>(getLength(pointCloud[ni] - position)));
		}
		return maxDist * 0.5;
	}
}

void MLSSurface::smooth(const double searchRadius)
{
	smoothedPoints = pointCloud;

	if (pointCloud.empty() || searchRadius <= 0.0) {
		return;
	}

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));
	for (const auto& p : pointCloud) spaceHash.add(p);

	const double sigma = searchRadius * 0.5;
	const auto size = pointCloud.size();
	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto indices = spaceHash.findNeighborIndices(i);
		const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
		if (!fit.valid) {
			continue; // leave the original position
		}

		const double f = fit.coeffs.back(); // height of the fitted surface at (u,v)=(0,0)
		const Vector3dd offset = fit.frame.normal * f;
		smoothedPoints[static_cast<size_t>(i)] = pointCloud[static_cast<size_t>(i)] + Vector3df(static_cast<float>(offset.x), static_cast<float>(offset.y), static_cast<float>(offset.z));
	}
}

void MLSSurface::smoothKNN(const size_t k)
{
	smoothedPoints = pointCloud;

	if (pointCloud.empty()) {
		return;
	}

	KDTree tree;
	tree.build(pointCloud);

	const auto size = pointCloud.size();
	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto indices = tree.findKNearestIndices(pointCloud[static_cast<size_t>(i)], k + 1);
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
		if (indices.size() > k) indices.resize(k);

		const double sigma = ::adaptiveSigma(pointCloud, static_cast<size_t>(i), indices);
		const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
		if (!fit.valid) {
			continue; // leave the original position
		}

		const double f = fit.coeffs.back(); // height of the fitted surface at (u,v)=(0,0)
		const Vector3dd offset = fit.frame.normal * f;
		smoothedPoints[static_cast<size_t>(i)] = pointCloud[static_cast<size_t>(i)] + Vector3df(static_cast<float>(offset.x), static_cast<float>(offset.y), static_cast<float>(offset.z));
	}
}

namespace {
	// Samples the given fitted surface on a regular grid within the tangent plane, out to
	// upsampleRadius and spaced stepSize apart, appending the results to output. Shared by
	// MLSSurface::upsample() and upsampleKNN().
	void appendUpsampledGrid(const Vector3df& position, const SurfaceFit& fit, const float upsampleRadius, const float stepSize, std::vector<Vector3df>& output) {
		const Vector3dd basePosition(position);
		for (float u = -upsampleRadius; u <= upsampleRadius; u += stepSize) {
			for (float w = -upsampleRadius; w <= upsampleRadius; w += stepSize) {
				if (u == 0.0f && w == 0.0f) {
					continue; // the original point itself, not a new sample
				}
				if (u * u + w * w > upsampleRadius * upsampleRadius) {
					continue;
				}
				const double h = evaluateHeightField(fit.coeffs, fit.order, static_cast<double>(u), static_cast<double>(w));
				const Vector3dd sample = basePosition + fit.frame.tangentU * static_cast<double>(u) + fit.frame.tangentV * static_cast<double>(w) + fit.frame.normal * h;
				output.emplace_back(static_cast<float>(sample.x), static_cast<float>(sample.y), static_cast<float>(sample.z));
			}
		}
	}
}

std::vector<Vector3df> MLSSurface::upsample(const double searchRadius, const float upsampleRadius, const float stepSize) const
{
	std::vector<Vector3df> output;
	if (pointCloud.empty() || searchRadius <= 0.0 || upsampleRadius <= 0.0f || stepSize <= 0.0f) {
		return output;
	}

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));
	for (const auto& p : pointCloud) spaceHash.add(p);

	const double sigma = searchRadius * 0.5;
	const auto size = pointCloud.size();
	const int n = static_cast<int>(size);

#ifdef _OPENMP
	// Each point's contribution is fully independent, but appends a variable number of samples,
	// so a shared `output` can't be written concurrently (emplace_back would race). Instead each
	// thread appends into its own buffer; since the default OpenMP schedule(static) hands out
	// contiguous, ascending ranges of `i` to threads in thread-index order, concatenating the
	// per-thread buffers in thread order reproduces the exact same point order as the serial loop.
	std::vector<std::vector<Vector3df>> perThread(static_cast<size_t>(omp_get_max_threads()));
#pragma omp parallel
	{
		auto& local = perThread[static_cast<size_t>(omp_get_thread_num())];
#pragma omp for nowait
		for (int i = 0; i < n; ++i) {
			const auto indices = spaceHash.findNeighborIndices(i);
			const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
			if (!fit.valid) {
				continue;
			}
			::appendUpsampledGrid(pointCloud[static_cast<size_t>(i)], fit, upsampleRadius, stepSize, local);
		}
	}
	for (const auto& local : perThread) {
		output.insert(output.end(), local.begin(), local.end());
	}
#else
	for (int i = 0; i < n; ++i) {
		const auto indices = spaceHash.findNeighborIndices(i);
		const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
		if (!fit.valid) {
			continue;
		}
		::appendUpsampledGrid(pointCloud[static_cast<size_t>(i)], fit, upsampleRadius, stepSize, output);
	}
#endif
	return output;
}

std::vector<Vector3df> MLSSurface::upsampleKNN(const size_t k, const float upsampleRadius, const float stepSize) const
{
	std::vector<Vector3df> output;
	if (pointCloud.empty() || upsampleRadius <= 0.0f || stepSize <= 0.0f) {
		return output;
	}

	KDTree tree;
	tree.build(pointCloud);

	const auto size = pointCloud.size();
	const int n = static_cast<int>(size);

#ifdef _OPENMP
	// See upsample() for why per-thread buffers (rather than a shared `output`) are required,
	// and why concatenating them in thread order preserves the original point ordering.
	std::vector<std::vector<Vector3df>> perThread(static_cast<size_t>(omp_get_max_threads()));
#pragma omp parallel
	{
		auto& local = perThread[static_cast<size_t>(omp_get_thread_num())];
#pragma omp for nowait
		for (int i = 0; i < n; ++i) {
			// +1 since the query point itself is typically returned as its own nearest neighbor.
			auto indices = tree.findKNearestIndices(pointCloud[static_cast<size_t>(i)], k + 1);
			indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
			if (indices.size() > k) indices.resize(k);

			const double sigma = ::adaptiveSigma(pointCloud, static_cast<size_t>(i), indices);
			const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
			if (!fit.valid) {
				continue;
			}
			::appendUpsampledGrid(pointCloud[static_cast<size_t>(i)], fit, upsampleRadius, stepSize, local);
		}
	}
	for (const auto& local : perThread) {
		output.insert(output.end(), local.begin(), local.end());
	}
#else
	for (int i = 0; i < n; ++i) {
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto indices = tree.findKNearestIndices(pointCloud[static_cast<size_t>(i)], k + 1);
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
		if (indices.size() > k) indices.resize(k);

		const double sigma = ::adaptiveSigma(pointCloud, static_cast<size_t>(i), indices);
		const auto fit = ::fitSurface(pointCloud, static_cast<size_t>(i), indices, sigma);
		if (!fit.valid) {
			continue;
		}
		::appendUpsampledGrid(pointCloud[static_cast<size_t>(i)], fit, upsampleRadius, stepSize, output);
	}
#endif
	return output;
}
