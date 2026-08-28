#include "CurvatureEstimator.h"
#include <CGLib/Space/Space/CompactSpaceHash.h>
#include <CGLib/Space/Space/KDTree.h>
#include <CGLib/Math/Matrix3d.h>
#include <CGLib/Numerics/Numerics/SVD3d.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>


using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::Numerics;
using namespace Phantom::PC;

namespace {
	Matrix3df calculateCovarianceMatrix(const std::vector<Vector3df>& points) {
		Vector3df means(0, 0, 0);

		for (const auto& p : points) {
			means += p;
		}

		means /= static_cast<float>(points.size());

		Matrix3df matrix;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix[i][j] = 0.0;
				for (int k = 0; k < points.size(); k++)
					matrix[i][j] += (means[i] - points[k][i]) *
					(means[j] - points[k][j]);
				matrix[i][j] /= points.size() - 1;
			}
		}
		return matrix;
	}

	// Double-precision variant (SVD3d::calculate requires Matrix3dd) used only by
	// CurvatureEstimator::estimatePrincipal(), which also needs the eigenvectors (not just the
	// eigenvalues) to build the PCA tangent frame.
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

	// Solves the 5x5 normal-equation system A*x = b (from least-squares quadric fitting) via
	// Gauss-Jordan elimination with partial pivoting. Returns false if A is (numerically) singular.
	bool solveLeastSquares5(double A[5][5], double b[5], double outX[5]) {
		for (int col = 0; col < 5; ++col) {
			int pivotRow = col;
			double pivotMag = std::abs(A[col][col]);
			for (int row = col + 1; row < 5; ++row) {
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
			for (int j = 0; j < 5; ++j) A[col][j] /= pivot;
			b[col] /= pivot;
			for (int row = 0; row < 5; ++row) {
				if (row == col) continue;
				const double factor = A[row][col];
				if (factor == 0.0) continue;
				for (int j = 0; j < 5; ++j) A[row][j] -= factor * A[col][j];
				b[row] -= factor * b[col];
			}
		}
		for (int i = 0; i < 5; ++i) outX[i] = b[i];
		return true;
	}

	// Scalar curvature at pointCloud[i] from its neighbor indices (self excluded), shared by
	// CurvatureEstimator::estimate() and estimateKNN().
	double fitCurvature(const std::vector<Vector3df>& pointCloud, size_t i, const std::vector<int>& indices) {
		const auto& position = pointCloud[i];
		std::vector<Vector3df> vectors;
		vectors.reserve(indices.size());
		for (const auto& pi : indices) {
			vectors.push_back(pointCloud[pi] - position);
		}
		if (vectors.size() < 3) {
			return 0.0;
		}
		const auto matrix = ::calculateCovarianceMatrix(vectors);
		SVD3d svd;
		const auto res = svd.calculate(matrix);
		const auto sum = res.eigenValues[0] + res.eigenValues[1] + res.eigenValues[2];
		return (sum > 0.0) ? res.eigenValues[0] / sum : 0.0;
	}

	// Principal curvatures/directions at pointCloud[i] from its neighbor indices (self excluded),
	// shared by CurvatureEstimator::estimatePrincipal() and estimatePrincipalKNN(). Returns a
	// zero-initialized PrincipalCurvature if there are fewer than 6 neighbors (needed to fit the
	// 5 quadric coefficients).
	CurvatureEstimator::PrincipalCurvature fitPrincipalCurvature(const std::vector<Vector3df>& pointCloud, size_t i, const std::vector<int>& indices) {
		CurvatureEstimator::PrincipalCurvature pc;
		if (indices.size() < 6) {
			return pc;
		}

		const auto& position = pointCloud[i];
		std::vector<Vector3df> vectors;
		vectors.reserve(indices.size());
		for (const auto pi : indices) {
			vectors.push_back(pointCloud[pi] - position);
		}

		const auto covariance = ::calculateCovarianceMatrixDouble(vectors);
		SVD3d svd;
		const auto res = svd.calculate(covariance);
		const Vector3dd normal = res.eigenVectors[0];
		const Vector3dd tangentU = res.eigenVectors[1];
		const Vector3dd tangentV = res.eigenVectors[2];

		// Least-squares fit of a Monge patch h = a*u^2+b*u*v+c*v^2+d*u+e*v in the (u,v,h) frame
		// spanned by (tangentU, tangentV, normal), via the normal equations A*x=b.
		double A[5][5] = {};
		double b[5] = {};
		for (const auto& v : vectors) {
			const Vector3dd vd(v);
			const double u = glm::dot(vd, tangentU);
			const double w = glm::dot(vd, tangentV);
			const double h = glm::dot(vd, normal);
			const double phi[5] = { u * u, u * w, w * w, u, w };
			for (int r = 0; r < 5; ++r) {
				for (int c = 0; c < 5; ++c) {
					A[r][c] += phi[r] * phi[c];
				}
				b[r] += phi[r] * h;
			}
		}

		double x[5] = {};
		if (!::solveLeastSquares5(A, b, x)) {
			return pc;
		}
		const double a = x[0], bCoef = x[1], c = x[2];

		// Principal curvatures/directions are the eigen-decomposition of the quadric's Hessian
		// [[2a,b],[b,2c]] -- the second fundamental form at the origin, since the fitted patch
		// passes through the point with the PCA normal as its exact surface normal there.
		const double hessA = 2.0 * a;
		const double hessB = bCoef;
		const double hessC = 2.0 * c;
		const double mean = (hessA + hessC) * 0.5;
		const double diff = (hessA - hessC) * 0.5;
		const double r = std::sqrt(diff * diff + hessB * hessB);
		const double k1 = mean + r;
		const double k2 = mean - r;

		double e1x, e1y;
		if (std::abs(hessB) > 1.0e-12) {
			e1x = hessB;
			e1y = k1 - hessA;
		} else if (hessA >= hessC) {
			e1x = 1.0; e1y = 0.0;
		} else {
			e1x = 0.0; e1y = 1.0;
		}
		const double e1Len = std::sqrt(e1x * e1x + e1y * e1y);
		if (e1Len > 1.0e-12) {
			e1x /= e1Len;
			e1y /= e1Len;
		}
		// The second eigenvector is the in-plane perpendicular (2x2 symmetric matrix).
		const double e2x = -e1y;
		const double e2y = e1x;

		pc.k1 = k1;
		pc.k2 = k2;
		const Vector3dd dir1 = tangentU * e1x + tangentV * e1y;
		const Vector3dd dir2 = tangentU * e2x + tangentV * e2y;
		pc.direction1 = Vector3df(static_cast<float>(dir1.x), static_cast<float>(dir1.y), static_cast<float>(dir1.z));
		pc.direction2 = Vector3df(static_cast<float>(dir2.x), static_cast<float>(dir2.y), static_cast<float>(dir2.z));
		return pc;
	}
}


void CurvatureEstimator::estimate(const double searchRadius)
{
	curvatures.clear();

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));

	const auto size = pointCloud.size();
	for (size_t i = 0; i < size; ++i) {
		spaceHash.add(pointCloud[i]);
	}

	curvatures.resize(size);
	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto indices = spaceHash.findNeighborIndices(i);
		curvatures[static_cast<size_t>(i)] = ::fitCurvature(pointCloud, static_cast<size_t>(i), indices);
	}
}

void CurvatureEstimator::estimateKNN(const size_t k)
{
	curvatures.clear();

	if (pointCloud.empty()) {
		return;
	}

	KDTree tree;
	tree.build(pointCloud);

	const auto size = pointCloud.size();
	curvatures.resize(size);
	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		// +1 since the query point itself is typically returned as its own nearest neighbor.
		auto indices = tree.findKNearestIndices(pointCloud[static_cast<size_t>(i)], k + 1);
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());
		if (indices.size() > k) indices.resize(k);

		curvatures[static_cast<size_t>(i)] = ::fitCurvature(pointCloud, static_cast<size_t>(i), indices);
	}
}

void CurvatureEstimator::estimatePrincipal(const double searchRadius)
{
	principalCurvatures.clear();
	principalCurvatures.resize(pointCloud.size());

	if (pointCloud.empty()) {
		return;
	}

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(pointCloud.size()));
	const auto size = pointCloud.size();
	for (size_t i = 0; i < size; ++i) {
		spaceHash.add(pointCloud[i]);
	}

	const int n = static_cast<int>(size);
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto indices = spaceHash.findNeighborIndices(i);
		principalCurvatures[static_cast<size_t>(i)] = ::fitPrincipalCurvature(pointCloud, static_cast<size_t>(i), indices);
	}
}

void CurvatureEstimator::estimatePrincipalKNN(const size_t k)
{
	principalCurvatures.clear();
	principalCurvatures.resize(pointCloud.size());

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

		principalCurvatures[static_cast<size_t>(i)] = ::fitPrincipalCurvature(pointCloud, static_cast<size_t>(i), indices);
	}
}
