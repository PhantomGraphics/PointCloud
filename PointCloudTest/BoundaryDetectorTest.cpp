#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/BoundaryDetector.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

// 11x11 flat grid in the XY plane (x,y in [-5,5]), spacing 1, all normals +Z.
// Index for (x,y) is (x+5)*11 + (y+5).
std::vector<Vector3df> makeGridPositions()
{
	std::vector<Vector3df> positions;
	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			positions.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
		}
	}
	return positions;
}

} // namespace

TEST(BoundaryDetectorTest, EmptyInputReturnsEmptyFlags)
{
	BoundaryDetector detector;
	detector.estimate(1.0);
	EXPECT_TRUE(detector.getBoundaryFlags().empty());
}

TEST(BoundaryDetectorTest, OutputSizeMatchesInputSize)
{
	BoundaryDetector detector;
	detector.add(Vector3df(0, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(1, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(2, 0, 0), Vector3df(0, 0, 1));

	detector.estimate(1.5);
	EXPECT_EQ(3u, detector.getBoundaryFlags().size());
}

TEST(BoundaryDetectorTest, TooFewNeighborsLeavesInterior)
{
	// Only 2 points total, 1.5 apart -- never enough neighbors (need >= 3) to judge.
	BoundaryDetector detector;
	detector.add(Vector3df(0, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(1, 0, 0), Vector3df(0, 0, 1));

	detector.estimate(1.5);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(2u, flags.size());
	EXPECT_FALSE(flags[0]);
	EXPECT_FALSE(flags[1]);
}

TEST(BoundaryDetectorTest, InteriorPointOfFlatGridIsNotBoundary)
{
	const auto positions = makeGridPositions();

	BoundaryDetector detector;
	for (const auto& p : positions) detector.add(p, Vector3df(0.0f, 0.0f, 1.0f));

	detector.estimate(1.5);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(positions.size(), flags.size());

	// Center point (0,0): full ring of 8 neighbors (4 axis + 4 diagonal), max angular gap 45 deg.
	const size_t centerIndex = (0 + 5) * 11 + (0 + 5);
	EXPECT_FALSE(flags[centerIndex]);
}

TEST(BoundaryDetectorTest, EdgePointOfFlatGridIsBoundary)
{
	const auto positions = makeGridPositions();

	BoundaryDetector detector;
	for (const auto& p : positions) detector.add(p, Vector3df(0.0f, 0.0f, 1.0f));

	detector.estimate(1.5);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(positions.size(), flags.size());

	// Edge-of-grid point (x=-5, y=0): neighbors only exist toward +x, leaving a ~180 degree gap.
	const size_t edgeIndex = (-5 + 5) * 11 + (0 + 5);
	EXPECT_TRUE(flags[edgeIndex]);

	// Corner point (x=-5, y=-5): neighbors confined to a single quadrant, an even bigger gap.
	const size_t cornerIndex = (-5 + 5) * 11 + (-5 + 5);
	EXPECT_TRUE(flags[cornerIndex]);
}

TEST(BoundaryDetectorTest, EstimateKNNEmptyInputReturnsEmptyFlags)
{
	BoundaryDetector detector;
	detector.estimateKNN(8);
	EXPECT_TRUE(detector.getBoundaryFlags().empty());
}

TEST(BoundaryDetectorTest, EstimateKNNOutputSizeMatchesInputSize)
{
	BoundaryDetector detector;
	detector.add(Vector3df(0, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(1, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(2, 0, 0), Vector3df(0, 0, 1));

	detector.estimateKNN(2);
	EXPECT_EQ(3u, detector.getBoundaryFlags().size());
}

TEST(BoundaryDetectorTest, EstimateKNNTooFewNeighborsLeavesInterior)
{
	// Only 2 points total -- never enough neighbors (need >= 3) to judge.
	BoundaryDetector detector;
	detector.add(Vector3df(0, 0, 0), Vector3df(0, 0, 1));
	detector.add(Vector3df(1, 0, 0), Vector3df(0, 0, 1));

	detector.estimateKNN(8);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(2u, flags.size());
	EXPECT_FALSE(flags[0]);
	EXPECT_FALSE(flags[1]);
}

TEST(BoundaryDetectorTest, EstimateKNNInteriorPointOfFlatGridIsNotBoundary)
{
	const auto positions = makeGridPositions();

	BoundaryDetector detector;
	for (const auto& p : positions) detector.add(p, Vector3df(0.0f, 0.0f, 1.0f));

	detector.estimateKNN(8);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(positions.size(), flags.size());

	// Center point (0,0): full ring of 8 neighbors (4 axis + 4 diagonal), max angular gap 45 deg.
	const size_t centerIndex = (0 + 5) * 11 + (0 + 5);
	EXPECT_FALSE(flags[centerIndex]);
}

TEST(BoundaryDetectorTest, EstimateKNNEdgePointOfFlatGridIsBoundary)
{
	const auto positions = makeGridPositions();

	BoundaryDetector detector;
	for (const auto& p : positions) detector.add(p, Vector3df(0.0f, 0.0f, 1.0f));

	detector.estimateKNN(8);
	const auto flags = detector.getBoundaryFlags();
	ASSERT_EQ(positions.size(), flags.size());

	// Edge-of-grid point (x=-5, y=0): neighbors only exist toward +x, leaving a ~180 degree gap.
	const size_t edgeIndex = (-5 + 5) * 11 + (0 + 5);
	EXPECT_TRUE(flags[edgeIndex]);

	// Corner point (x=-5, y=-5): neighbors confined to a single quadrant, an even bigger gap.
	const size_t cornerIndex = (-5 + 5) * 11 + (-5 + 5);
	EXPECT_TRUE(flags[cornerIndex]);
}
