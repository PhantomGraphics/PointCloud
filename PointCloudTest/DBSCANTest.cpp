#include "pch.h"
#include "../PointCloud/DBSCAN.h"

using namespace Phantom::PC;

// Two dense groups far apart: each point has enough neighbors (excluding
// itself) to be a core point, so each group collapses into one cluster and
// the two clusters get different IDs.
TEST(DBSCANTest, TwoDenseGroupsFormSeparateClusters)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.05, 0.0, 0.0),
		Point(0.0, 0.05, 0.0), Point(0.05, 0.05, 0.0),

		Point(10.0, 10.0, 10.0), Point(10.05, 10.0, 10.0),
		Point(10.0, 10.05, 10.0), Point(10.05, 10.05, 10.0),
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/0.1, /*minPts=*/3);

	for (const auto& p : data) {
		EXPECT_GT(p.clusterID, 0);
	}

	const int clusterA = data[0].clusterID;
	EXPECT_EQ(clusterA, data[1].clusterID);
	EXPECT_EQ(clusterA, data[2].clusterID);
	EXPECT_EQ(clusterA, data[3].clusterID);

	const int clusterB = data[4].clusterID;
	EXPECT_EQ(clusterB, data[5].clusterID);
	EXPECT_EQ(clusterB, data[6].clusterID);
	EXPECT_EQ(clusterB, data[7].clusterID);

	EXPECT_NE(clusterA, clusterB);
}

// A point with no neighbors within eps can never satisfy minPts and is never
// reached by another cluster's expansion, so it remains permanent noise (-1).
TEST(DBSCANTest, IsolatedPointBecomesNoise)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.05, 0.0, 0.0), Point(0.0, 0.05, 0.0),
		Point(100.0, 100.0, 100.0), // far isolated point
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/0.1, /*minPts=*/2);

	EXPECT_GT(data[0].clusterID, 0);
	EXPECT_GT(data[1].clusterID, 0);
	EXPECT_GT(data[2].clusterID, 0);
	EXPECT_EQ(-1, data[3].clusterID);
}

// minPts counts neighbors excluding the point itself. With minPts higher than
// the available neighbor count, every point stays noise even though they are
// mutually close.
TEST(DBSCANTest, MinPtsTooHighLeavesAllPointsAsNoise)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.05, 0.0, 0.0), Point(0.0, 0.05, 0.0),
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/0.1, /*minPts=*/5);

	for (const auto& p : data) {
		EXPECT_EQ(-1, p.clusterID);
	}
}

// The underlying spatial hash (CompactSpaceHash) buckets neighbors using a
// strict "<" distance check against eps, so two points exactly eps apart are
// NOT found as neighbors of each other (unlike a naive "<=" reading of the
// eps parameter might suggest).
TEST(DBSCANTest, PointsExactlyAtEpsDistanceAreNotNeighbors)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0),
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/1.0, /*minPts=*/1);

	EXPECT_EQ(-1, data[0].clusterID);
	EXPECT_EQ(-1, data[1].clusterID);
}

// A point strictly closer than eps is found as a neighbor, confirming the
// exclusive boundary demonstrated above is specifically about equality.
TEST(DBSCANTest, PointsStrictlyWithinEpsDistanceAreNeighbors)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.99, 0.0, 0.0),
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/1.0, /*minPts=*/1);

	EXPECT_GT(data[0].clusterID, 0);
	EXPECT_EQ(data[0].clusterID, data[1].clusterID);
}

// F-3: a ProgressReporter that always continues must not change the clustering
// result, and must observe progress climbing monotonically up to a final 1.0
// report emitted once the outer loop finishes without being cancelled.
TEST(DBSCANTest, ProgressReporterReportsMonotonicallyToCompletionWithoutCancelling)
{
	std::vector<Point> data = {
		Point(0.0, 0.0, 0.0), Point(0.05, 0.0, 0.0),
		Point(0.0, 0.05, 0.0), Point(0.05, 0.05, 0.0),

		Point(10.0, 10.0, 10.0), Point(10.05, 10.0, 10.0),
		Point(10.0, 10.05, 10.0), Point(10.05, 10.05, 10.0),
	};

	std::vector<float> reported;
	ProgressReporter reporter;
	reporter.callback = [&](float progress) -> bool {
		reported.push_back(progress);
		return true;
	};

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/0.1, /*minPts=*/3, reporter);

	// Same outcome as TwoDenseGroupsFormSeparateClusters: the reporter must not
	// perturb the algorithm when it always allows continuation.
	const int clusterA = data[0].clusterID;
	const int clusterB = data[4].clusterID;
	EXPECT_GT(clusterA, 0);
	EXPECT_GT(clusterB, 0);
	EXPECT_NE(clusterA, clusterB);

	ASSERT_FALSE(reported.empty());
	for (size_t i = 1; i < reported.size(); ++i) {
		EXPECT_GE(reported[i], reported[i - 1]);
	}
	EXPECT_FLOAT_EQ(reported.front(), 0.0f);
	EXPECT_FLOAT_EQ(reported.back(), 1.0f);
}

// F-3: returning false from the callback must stop the outer loop before it
// reaches every point (not just be ignored), and any point left unclassified
// by the early exit must be finalized as noise (-1) so the returned labels
// remain a structurally valid (if incomplete) clustering.
TEST(DBSCANTest, ProgressReporterCancellationStopsLoopEarlyAndMarksRestAsNoise)
{
	// Ten mutually far-apart points: minPts is unreachable, so each outer
	// iteration classifies exactly one point (no BFS expansion across points),
	// making the callback invocation count equal to the number of points visited.
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

	DBSCANClustering clustering;
	clustering.cluster(data, /*eps=*/1.0, /*minPts=*/99, reporter);

	EXPECT_EQ(callCount, 4);
	for (const auto& p : data) {
		EXPECT_EQ(-1, p.clusterID);
	}
}
