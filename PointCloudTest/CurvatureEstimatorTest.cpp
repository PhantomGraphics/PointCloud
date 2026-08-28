#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/CurvatureEstimator.h"
#include "../../CGLib/Math/Vector3d.h"

#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

// Single point has no neighbors, so curvature should be 0
TEST(CurvatureEstimatorTest, SinglePoint)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));

	estimator.estimate(1.0);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(1u, curvatures.size());
	EXPECT_NEAR(0.0, curvatures[0], 1e-6);
}

// Output size must match the number of input points
TEST(CurvatureEstimatorTest, OutputSizeMatchesInputSize)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));

	estimator.estimate(1.5);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(3u, curvatures.size());
}

// Points on a flat XY plane: the center point curvature should be near 0
// because the smallest eigenvalue of the covariance matrix approaches 0
TEST(CurvatureEstimatorTest, FlatPlane)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df( 0.0f,  0.0f, 0.0f));  // center (index 0)
	estimator.add(Vector3df( 1.0f,  0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f,  0.0f, 0.0f));
	estimator.add(Vector3df( 0.0f,  1.0f, 0.0f));
	estimator.add(Vector3df( 0.0f, -1.0f, 0.0f));

	estimator.estimate(1.5);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(5u, curvatures.size());
	EXPECT_NEAR(0.0, curvatures[0], 1e-5);
}

// Points evenly placed along all 6 axis directions: curvature should be 1/3
// Covariance matrix has equal eigenvalues -> sigma = lambda / (3 * lambda) = 1/3
TEST(CurvatureEstimatorTest, IsotropicDistribution)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df( 0.0f,  0.0f,  0.0f));  // center (index 0)
	estimator.add(Vector3df( 1.0f,  0.0f,  0.0f));
	estimator.add(Vector3df(-1.0f,  0.0f,  0.0f));
	estimator.add(Vector3df( 0.0f,  1.0f,  0.0f));
	estimator.add(Vector3df( 0.0f, -1.0f,  0.0f));
	estimator.add(Vector3df( 0.0f,  0.0f,  1.0f));
	estimator.add(Vector3df( 0.0f,  0.0f, -1.0f));

	estimator.estimate(1.5);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(7u, curvatures.size());
	EXPECT_NEAR(1.0 / 3.0, curvatures[0], 1e-5);
}

// Curvature values must always lie in the range [0, 1/3]
TEST(CurvatureEstimatorTest, CurvatureInValidRange)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.5f, 0.2f));
	estimator.add(Vector3df(0.3f, 1.0f, 0.1f));
	estimator.add(Vector3df(0.8f, 0.2f, 1.0f));
	estimator.add(Vector3df(0.5f, 0.7f, 0.9f));

	estimator.estimate(2.0);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(5u, curvatures.size());
	for (const auto& c : curvatures) {
		EXPECT_GE(c, 0.0);
		EXPECT_LE(c, 1.0 / 3.0 + 1e-9);
	}
}

// Output size must match the number of input points, even for points too sparse to fit a quadric.
TEST(CurvatureEstimatorTest, EstimatePrincipal_OutputSizeMatchesInputSize)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));

	estimator.estimatePrincipal(1.5);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(3u, pc.size());
}

// Fewer than 6 neighbors can't determine the 5 quadric coefficients: left zero-initialized.
TEST(CurvatureEstimatorTest, EstimatePrincipal_TooFewNeighborsLeavesZeroed)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));  // center (index 0)
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));  // only 4 neighbors, need >= 6

	estimator.estimatePrincipal(1.5);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(5u, pc.size());
	EXPECT_NEAR(0.0, pc[0].k1, 1e-9);
	EXPECT_NEAR(0.0, pc[0].k2, 1e-9);
	EXPECT_NEAR(0.0f, getLength(pc[0].direction1), 1e-6f);
	EXPECT_NEAR(0.0f, getLength(pc[0].direction2), 1e-6f);
}

// A flat 5x5 grid (8 neighbors within radius 1.5) has zero curvature in every direction.
TEST(CurvatureEstimatorTest, EstimatePrincipal_FlatPlaneHasNearZeroPrincipalCurvatures)
{
	CurvatureEstimator estimator;
	for (int x = -2; x <= 2; ++x) {
		for (int y = -2; y <= 2; ++y) {
			estimator.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
		}
	}

	estimator.estimatePrincipal(1.5);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(25u, pc.size());

	// Center point is index (0,0) -> (0+2)*5+(0+2) = 12.
	EXPECT_NEAR(0.0, pc[12].k1, 1e-6);
	EXPECT_NEAR(0.0, pc[12].k2, 1e-6);
}

// Points densely sampled around the north pole of a sphere of radius R are umbilic: both
// principal curvatures should be close to +-1/R (the sign depends on which way the PCA normal
// happens to point, but |k1| ~= |k2| ~= 1/R regardless), and the fitted principal directions
// should remain an orthonormal tangent-plane basis.
TEST(CurvatureEstimatorTest, EstimatePrincipal_SphereProducesInverseRadiusCurvatures)
{
	const float radius = 2.0f;

	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, radius));  // pole point, index 0

	for (int ring = 1; ring <= 6; ++ring) {
		const float theta = ring * 0.05f;
		for (int a = 0; a < 16; ++a) {
			const float phi = a * (2.0f * 3.14159265358979323846f / 16.0f);
			estimator.add(Vector3df(
				radius * std::sin(theta) * std::cos(phi),
				radius * std::sin(theta) * std::sin(phi),
				radius * std::cos(theta)));
		}
	}

	estimator.estimatePrincipal(0.65);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_FALSE(pc.empty());

	const auto& pole = pc[0];
	const double expected = 1.0 / radius;
	EXPECT_NEAR(std::abs(pole.k1), expected, 0.05);
	EXPECT_NEAR(std::abs(pole.k2), expected, 0.05);
	EXPECT_NEAR(pole.k1, pole.k2, 0.05); // umbilic: same sign, same magnitude

	EXPECT_NEAR(1.0f, getLength(pole.direction1), 1.0e-3f);
	EXPECT_NEAR(1.0f, getLength(pole.direction2), 1.0e-3f);
	EXPECT_NEAR(0.0f, glm::dot(pole.direction1, pole.direction2), 1.0e-3f);
}

// Single point has no neighbors, so curvature should be 0
TEST(CurvatureEstimatorTest, EstimateKNNSinglePoint)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));

	estimator.estimateKNN(4);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(1u, curvatures.size());
	EXPECT_NEAR(0.0, curvatures[0], 1e-6);
}

// Output size must match the number of input points
TEST(CurvatureEstimatorTest, EstimateKNNOutputSizeMatchesInputSize)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));

	estimator.estimateKNN(2);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(3u, curvatures.size());
}

// Points on a flat XY plane: the center point curvature should be near 0
// because the smallest eigenvalue of the covariance matrix approaches 0
TEST(CurvatureEstimatorTest, EstimateKNNFlatPlane)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df( 0.0f,  0.0f, 0.0f));  // center (index 0)
	estimator.add(Vector3df( 1.0f,  0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f,  0.0f, 0.0f));
	estimator.add(Vector3df( 0.0f,  1.0f, 0.0f));
	estimator.add(Vector3df( 0.0f, -1.0f, 0.0f));

	estimator.estimateKNN(4);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(5u, curvatures.size());
	EXPECT_NEAR(0.0, curvatures[0], 1e-5);
}

// Points evenly placed along all 6 axis directions: curvature should be 1/3
TEST(CurvatureEstimatorTest, EstimateKNNIsotropicDistribution)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df( 0.0f,  0.0f,  0.0f));  // center (index 0)
	estimator.add(Vector3df( 1.0f,  0.0f,  0.0f));
	estimator.add(Vector3df(-1.0f,  0.0f,  0.0f));
	estimator.add(Vector3df( 0.0f,  1.0f,  0.0f));
	estimator.add(Vector3df( 0.0f, -1.0f,  0.0f));
	estimator.add(Vector3df( 0.0f,  0.0f,  1.0f));
	estimator.add(Vector3df( 0.0f,  0.0f, -1.0f));

	estimator.estimateKNN(6);

	const auto curvatures = estimator.getCurvatures();
	ASSERT_EQ(7u, curvatures.size());
	EXPECT_NEAR(1.0 / 3.0, curvatures[0], 1e-5);
}

// Output size must match the number of input points, even for points too sparse to fit a quadric.
TEST(CurvatureEstimatorTest, EstimatePrincipalKNN_OutputSizeMatchesInputSize)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));

	estimator.estimatePrincipalKNN(2);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(3u, pc.size());
}

// Fewer than 6 neighbors can't determine the 5 quadric coefficients: left zero-initialized.
// Only 4 other points exist, so even requesting k=4 leaves indices.size() == 4 < 6.
TEST(CurvatureEstimatorTest, EstimatePrincipalKNN_TooFewNeighborsLeavesZeroed)
{
	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));  // center (index 0)
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));  // only 4 other points exist

	estimator.estimatePrincipalKNN(4);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(5u, pc.size());
	EXPECT_NEAR(0.0, pc[0].k1, 1e-9);
	EXPECT_NEAR(0.0, pc[0].k2, 1e-9);
	EXPECT_NEAR(0.0f, getLength(pc[0].direction1), 1e-6f);
	EXPECT_NEAR(0.0f, getLength(pc[0].direction2), 1e-6f);
}

// A flat 5x5 grid (8 neighbors for the center point) has zero curvature in every direction.
TEST(CurvatureEstimatorTest, EstimatePrincipalKNN_FlatPlaneHasNearZeroPrincipalCurvatures)
{
	CurvatureEstimator estimator;
	for (int x = -2; x <= 2; ++x) {
		for (int y = -2; y <= 2; ++y) {
			estimator.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
		}
	}

	estimator.estimatePrincipalKNN(8);

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_EQ(25u, pc.size());

	// Center point is index (0,0) -> (0+2)*5+(0+2) = 12.
	EXPECT_NEAR(0.0, pc[12].k1, 1e-6);
	EXPECT_NEAR(0.0, pc[12].k2, 1e-6);
}

// Same sphere-cap setup as EstimatePrincipal_SphereProducesInverseRadiusCurvatures, but requests
// all other points as k nearest neighbors (equivalent to the radius search there, since every
// added point already lies within that test's search radius).
TEST(CurvatureEstimatorTest, EstimatePrincipalKNN_SphereProducesInverseRadiusCurvatures)
{
	const float radius = 2.0f;

	CurvatureEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, radius));  // pole point, index 0

	for (int ring = 1; ring <= 6; ++ring) {
		const float theta = ring * 0.05f;
		for (int a = 0; a < 16; ++a) {
			const float phi = a * (2.0f * 3.14159265358979323846f / 16.0f);
			estimator.add(Vector3df(
				radius * std::sin(theta) * std::cos(phi),
				radius * std::sin(theta) * std::sin(phi),
				radius * std::cos(theta)));
		}
	}

	estimator.estimatePrincipalKNN(96); // all other points (1 + 6*16 = 97 total)

	const auto pc = estimator.getPrincipalCurvatures();
	ASSERT_FALSE(pc.empty());

	const auto& pole = pc[0];
	const double expected = 1.0 / radius;
	EXPECT_NEAR(std::abs(pole.k1), expected, 0.05);
	EXPECT_NEAR(std::abs(pole.k2), expected, 0.05);
	EXPECT_NEAR(pole.k1, pole.k2, 0.05); // umbilic: same sign, same magnitude

	EXPECT_NEAR(1.0f, getLength(pole.direction1), 1.0e-3f);
	EXPECT_NEAR(1.0f, getLength(pole.direction2), 1.0e-3f);
	EXPECT_NEAR(0.0f, glm::dot(pole.direction1, pole.direction2), 1.0e-3f);
}
