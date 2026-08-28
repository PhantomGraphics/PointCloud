#include "pch.h"
#include "../PointCloud/RegionGrowing.h"

#include <algorithm>
#include <vector>

using namespace Phantom::PC;
using namespace Phantom::Math;

TEST(RegionGrowingTest, FailsWithEmptyInput)
{
	RegionGrowing rg;
	EXPECT_FALSE(rg.segment());
	EXPECT_TRUE(rg.getLabels().empty());
	EXPECT_EQ(0u, rg.getClusterCount());
}

// Two flat, identically-oriented 11x11 grids far apart in space: spatial disconnection alone
// (via the k-NN neighbor search never crossing between them) must produce 2 separate clusters,
// even though nothing about their normals/curvature would prevent merging.
TEST(RegionGrowingTest, SpatiallySeparatedFlatPatchesFormTwoClusters)
{
	RegionGrowing rg;
	const Vector3df normal(0.0f, 0.0f, 1.0f);

	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			rg.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f), normal, 0.0);
		}
	}
	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			rg.add(Vector3df(static_cast<float>(x) + 100.0f, static_cast<float>(y), 0.0f), normal, 0.0);
		}
	}

	RegionGrowing::Params params;
	params.kNeighbors = 8;
	params.minClusterSize = 10;
	ASSERT_TRUE(rg.segment(params));

	EXPECT_EQ(2u, rg.getClusterCount());
	const auto labels = rg.getLabels();
	ASSERT_EQ(242u, labels.size());

	const int labelA = labels[0];
	const int labelB = labels[121];
	EXPECT_NE(-1, labelA);
	EXPECT_NE(-1, labelB);
	EXPECT_NE(labelA, labelB);
	for (size_t i = 0; i < 121; ++i) EXPECT_EQ(labelA, labels[i]);
	for (size_t i = 121; i < 242; ++i) EXPECT_EQ(labelB, labels[i]);
}

// Two adjacent (touching) flat patches meeting at a 90-degree fold: spatially contiguous enough
// that k-NN crosses the boundary, but the normal-smoothness test must still keep them separate.
TEST(RegionGrowingTest, SharpFoldBetweenAdjacentPatchesStaysTwoClusters)
{
	RegionGrowing rg;
	const Vector3df normalA(0.0f, 0.0f, 1.0f);
	const Vector3df normalB(1.0f, 0.0f, 0.0f);

	size_t countA = 0, countB = 0;
	for (int x = -5; x <= -1; ++x) {
		for (int y = -5; y <= 5; ++y) {
			rg.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f), normalA, 0.0);
			++countA;
		}
	}
	for (int x = 0; x <= 4; ++x) {
		for (int y = -5; y <= 5; ++y) {
			rg.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f), normalB, 0.0);
			++countB;
		}
	}

	RegionGrowing::Params params;
	params.kNeighbors = 8;
	params.minClusterSize = 10;
	ASSERT_TRUE(rg.segment(params));

	EXPECT_EQ(2u, rg.getClusterCount());
	const auto labels = rg.getLabels();
	ASSERT_EQ(countA + countB, labels.size());

	const int labelA = labels[0];
	const int labelB = labels[countA];
	EXPECT_NE(-1, labelA);
	EXPECT_NE(-1, labelB);
	EXPECT_NE(labelA, labelB);
	for (size_t i = 0; i < countA; ++i) EXPECT_EQ(labelA, labels[i]);
	for (size_t i = countA; i < countA + countB; ++i) EXPECT_EQ(labelB, labels[i]);
}

TEST(RegionGrowingTest, SmallRegionBelowMinClusterSizeStaysUnlabeled)
{
	RegionGrowing rg;
	const Vector3df normal(0.0f, 0.0f, 1.0f);
	// Only 3 points, isolated: never reaches a reasonable minClusterSize.
	rg.add(Vector3df(0.0f, 0.0f, 0.0f), normal, 0.0);
	rg.add(Vector3df(1.0f, 0.0f, 0.0f), normal, 0.0);
	rg.add(Vector3df(0.0f, 1.0f, 0.0f), normal, 0.0);

	RegionGrowing::Params params;
	params.kNeighbors = 8;
	params.minClusterSize = 10;
	ASSERT_TRUE(rg.segment(params));

	EXPECT_EQ(0u, rg.getClusterCount());
	const auto labels = rg.getLabels();
	ASSERT_EQ(3u, labels.size());
	for (int l : labels) EXPECT_EQ(-1, l);
}
