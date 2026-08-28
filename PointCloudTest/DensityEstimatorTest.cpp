#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/DensityEstimator.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

// A single point has no neighbors (self is excluded), so density must be 0.
TEST(DensityEstimatorTest, SinglePoint)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));

	const double searchRadius = 1.0;
	estimator.estimate(searchRadius);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(1u, densities.size());
	EXPECT_NEAR(0.0, densities[0], 1e-3);
}

// No points added: density list must be empty after estimate.
TEST(DensityEstimatorTest, EmptyInput)
{
	DensityEstimator estimator;
	estimator.estimate(1.0);
	EXPECT_TRUE(estimator.getDensities().empty());
}

// Output size must equal the number of added points.
TEST(DensityEstimatorTest, OutputSizeMatchesInput)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));
	estimator.estimate(0.5);
	EXPECT_EQ(3u, estimator.getDensities().size());
}

// Two points within the search radius: each must have a positive density
// because it receives a Gaussian contribution from the other point.
TEST(DensityEstimatorTest, TwoClosePointsHavePositiveDensity)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.1f, 0.0f, 0.0f)); // distance 0.1 < searchRadius 1.0

	estimator.estimate(1.0);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(2u, densities.size());
	EXPECT_GT(densities[0], 0.0);
	EXPECT_GT(densities[1], 0.0);
}

// Two points outside the search radius: neither is a neighbor of the other,
// so both densities must remain 0.
TEST(DensityEstimatorTest, TwoFarPointsHaveZeroDensity)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(10.0f, 0.0f, 0.0f)); // distance 10 >> searchRadius 1.0

	estimator.estimate(1.0);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(2u, densities.size());
	EXPECT_NEAR(0.0, densities[0], 1e-6);
	EXPECT_NEAR(0.0, densities[1], 1e-6);
}

// A denser cluster must produce higher densities than an isolated point.
TEST(DensityEstimatorTest, DenseClusterHigherThanIsolated)
{
	DensityEstimator estimator;
	// Dense cluster
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.1f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.2f, 0.0f, 0.0f));
	// Isolated point far away
	estimator.add(Vector3df(100.0f, 0.0f, 0.0f));

	estimator.estimate(1.0);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(4u, densities.size());
	// Cluster center (index 1) should have higher density than the isolated point.
	EXPECT_GT(densities[1], densities[3]);
	// Isolated point has no neighbors.
	EXPECT_NEAR(0.0, densities[3], 1e-6);
}

TEST(DensityEstimatorTest, EstimateDoesNotCarryOverPreviousValues)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.1f, 0.0f, 0.0f));

	estimator.estimate(1.0);
	const auto first = estimator.getDensities();
	ASSERT_EQ(2u, first.size());

	estimator.estimate(1.0);
	const auto second = estimator.getDensities();
	ASSERT_EQ(2u, second.size());

	EXPECT_NEAR(first[0], second[0], 1e-6);
	EXPECT_NEAR(first[1], second[1], 1e-6);
}

// A single point has no neighbors (self is excluded), so density must be 0.
TEST(DensityEstimatorTest, EstimateKNNSinglePoint)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));

	estimator.estimateKNN(4);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(1u, densities.size());
	EXPECT_NEAR(0.0, densities[0], 1e-3);
}

// No points added: density list must be empty after estimateKNN.
TEST(DensityEstimatorTest, EstimateKNNEmptyInput)
{
	DensityEstimator estimator;
	estimator.estimateKNN(4);
	EXPECT_TRUE(estimator.getDensities().empty());
}

// Output size must equal the number of added points.
TEST(DensityEstimatorTest, EstimateKNNOutputSizeMatchesInput)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(2.0f, 0.0f, 0.0f));
	estimator.estimateKNN(2);
	EXPECT_EQ(3u, estimator.getDensities().size());
}

// Every point has at least one neighbor (k=1), so every density must be positive.
TEST(DensityEstimatorTest, EstimateKNNTwoClosePointsHavePositiveDensity)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.1f, 0.0f, 0.0f));

	estimator.estimateKNN(1);

	const auto densities = estimator.getDensities();
	ASSERT_EQ(2u, densities.size());
	EXPECT_GT(densities[0], 0.0);
	EXPECT_GT(densities[1], 0.0);
}

// Unlike the fixed-radius estimate(), estimateKNN() normalizes its Gaussian kernel by each
// point's own k-th neighbor distance, so absolute cluster scale doesn't by itself change the
// result -- what's verified here is repeatability, mirroring EstimateDoesNotCarryOverPreviousValues.
TEST(DensityEstimatorTest, EstimateKNNDoesNotCarryOverPreviousValues)
{
	DensityEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.1f, 0.0f, 0.0f));

	estimator.estimateKNN(1);
	const auto first = estimator.getDensities();
	ASSERT_EQ(2u, first.size());

	estimator.estimateKNN(1);
	const auto second = estimator.getDensities();
	ASSERT_EQ(2u, second.size());

	EXPECT_NEAR(first[0], second[0], 1e-6);
	EXPECT_NEAR(first[1], second[1], 1e-6);
}
