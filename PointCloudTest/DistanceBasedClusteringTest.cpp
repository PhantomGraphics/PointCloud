#include "pch.h"
#include "../PointCloud/DistanceBasedClustering.h"

using namespace Phantom::PC;

// Groups 1 (x~1) and 2 (x~2.5) are ~1.5 apart.
// With radius=1.6 they are connected into a single cluster;
// Group 3 (x=10) is isolated and forms a separate cluster.
TEST(DistanceBasedClusteringTest, TwoGroupsMergeOneIsolated)
{
	std::vector<Point> data = {
		Point(1.0, 1.0, 1.0), Point(1.1, 1.0, 1.0),   // Group 1
		Point(2.5, 1.0, 1.0), Point(2.6, 1.0, 1.0),   // Group 2
		Point(10.0, 10.0, 10.0), Point(10.1, 10.0, 10.0) // Group 3
	};

	DistanceBasedClustering clustering;
	clustering.DistanceBasedRegionGrowing(data, 1.6);

	// All 6 points must be assigned (clusterID > 0).
	for (const auto& p : data) {
		EXPECT_GT(p.clusterID, 0);
	}

	// Groups 1 and 2 must share the same cluster ID.
	const int clusterA = data[0].clusterID;
	EXPECT_EQ(clusterA, data[1].clusterID);
	EXPECT_EQ(clusterA, data[2].clusterID);
	EXPECT_EQ(clusterA, data[3].clusterID);

	// Group 3 must have a different cluster ID.
	const int clusterB = data[4].clusterID;
	EXPECT_EQ(clusterB, data[5].clusterID);
	EXPECT_NE(clusterA, clusterB);

	// Exactly 2 distinct clusters.
	EXPECT_EQ(1, clusterA);
	EXPECT_EQ(2, clusterB);
}

// With a small radius, no points reach across groups: each pair forms its own cluster.
TEST(DistanceBasedClusteringTest, SmallRadiusProducesSeparateClusters)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.05, 0.0, 0.0),  // close pair A
		Point(5.0, 0.0, 0.0), Point(5.05, 0.0, 0.0),  // close pair B
	};

	DistanceBasedClustering clustering;
	clustering.DistanceBasedRegionGrowing(data, 0.1); // radius 0.1: pairs stay together but groups are separate

	// All points assigned.
	for (const auto& p : data) {
		EXPECT_GT(p.clusterID, 0);
	}

	// Within each pair, cluster ID must match.
	EXPECT_EQ(data[0].clusterID, data[1].clusterID);
	EXPECT_EQ(data[2].clusterID, data[3].clusterID);

	// The two pairs must be in different clusters.
	EXPECT_NE(data[0].clusterID, data[2].clusterID);
}

// All points very close together: all must end up in a single cluster.
TEST(DistanceBasedClusteringTest, AllPointsCloseFormOneCluster)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0),
		Point(0.1, 0.0, 0.0),
		Point(0.2, 0.0, 0.0),
		Point(0.3, 0.0, 0.0),
	};

	DistanceBasedClustering clustering;
	clustering.DistanceBasedRegionGrowing(data, 0.5);

	const int expectedCluster = data[0].clusterID;
	EXPECT_GT(expectedCluster, 0);
	for (const auto& p : data) {
		EXPECT_EQ(expectedCluster, p.clusterID);
	}
}

// F-3: a ProgressReporter that always continues must not change the clustering
// result, and must observe progress climbing monotonically up to a final 1.0
// report emitted once the outer loop finishes without being cancelled.
TEST(DistanceBasedClusteringTest, ProgressReporterReportsMonotonicallyToCompletionWithoutCancelling)
{
	std::vector<Point> data = {
		Point(1.0, 1.0, 1.0), Point(1.1, 1.0, 1.0),
		Point(2.5, 1.0, 1.0), Point(2.6, 1.0, 1.0),
		Point(10.0, 10.0, 10.0), Point(10.1, 10.0, 10.0),
	};

	std::vector<float> reported;
	ProgressReporter reporter;
	reporter.callback = [&](float progress) -> bool {
		reported.push_back(progress);
		return true;
	};

	DistanceBasedClustering clustering;
	clustering.DistanceBasedRegionGrowing(data, 1.6, reporter);

	// Same outcome as TwoGroupsMergeOneIsolated: the reporter must not perturb
	// the algorithm when it always allows continuation.
	EXPECT_EQ(1, data[0].clusterID);
	EXPECT_EQ(2, data[4].clusterID);

	ASSERT_FALSE(reported.empty());
	for (size_t i = 1; i < reported.size(); ++i) {
		EXPECT_GE(reported[i], reported[i - 1]);
	}
	EXPECT_FLOAT_EQ(reported.front(), 0.0f);
	EXPECT_FLOAT_EQ(reported.back(), 1.0f);
}

// F-3: returning false from the callback must stop the outer loop before it
// reaches every point. Any point left unclassified (clusterID == 0) by the
// early exit is finalized as -1 -- a value this algorithm never produces on
// its own (every reached point gets a positive cluster ID) -- so cancellation
// is distinguishable from a normal run purely by inspecting the labels.
TEST(DistanceBasedClusteringTest, ProgressReporterCancellationStopsLoopEarlyAndLeavesRestUnclassified)
{
	// Ten mutually far-apart points with a small radius: each outer iteration
	// seeds and immediately finishes its own single-point cluster (no BFS
	// expansion across points), so the callback invocation count equals the
	// number of points visited.
	std::vector<Point> data;
	for (int i = 0; i < 10; ++i) {
		data.emplace_back(static_cast<double>(i) * 100.0, 0.0, 0.0);
	}

	int callCount = 0;
	ProgressReporter reporter;
	reporter.callback = [&](float /*progress*/) -> bool {
		++callCount;
		return callCount < 4; // cancel on the 4th call
	};

	DistanceBasedClustering clustering;
	clustering.DistanceBasedRegionGrowing(data, /*searchRadius=*/1.0, reporter);

	EXPECT_EQ(callCount, 4);
	for (int i = 0; i < 3; ++i) {
		EXPECT_GT(data[i].clusterID, 0); // visited before cancellation
	}
	for (int i = 3; i < 10; ++i) {
		EXPECT_EQ(-1, data[i].clusterID); // never reached, finalized as -1
	}
}
