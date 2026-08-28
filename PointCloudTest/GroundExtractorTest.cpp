#include "pch.h"
#include "../PointCloud/GroundExtractor.h"

#include <vector>

using namespace Phantom::PC;
using namespace Phantom::Math;

TEST(GroundExtractorTest, FailsWithEmptyInput)
{
	GroundExtractor extractor;
	EXPECT_FALSE(extractor.extract());
	EXPECT_TRUE(extractor.getGroundFlags().empty());
}

TEST(GroundExtractorTest, FailsWithNonPositiveCellSize)
{
	GroundExtractor extractor;
	extractor.add(Vector3df(0, 0, 0));
	extractor.add(Vector3df(1, 0, 0));

	GroundExtractor::Params params;
	params.cellSize = 0.0f;
	EXPECT_FALSE(extractor.extract(params));
}

TEST(GroundExtractorTest, FailsWithGrowthFactorNotAboveOne)
{
	GroundExtractor extractor;
	extractor.add(Vector3df(0, 0, 0));
	extractor.add(Vector3df(1, 0, 0));

	GroundExtractor::Params params;
	params.windowGrowthFactor = 1.0f;
	EXPECT_FALSE(extractor.extract(params));
}

TEST(GroundExtractorTest, FlatTerrainIsAllGround)
{
	GroundExtractor extractor;
	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			extractor.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
		}
	}

	GroundExtractor::Params params;
	params.cellSize = 1.0f;
	ASSERT_TRUE(extractor.extract(params));

	const auto flags = extractor.getGroundFlags();
	ASSERT_EQ(121u, flags.size());
	for (bool f : flags) {
		EXPECT_TRUE(f);
	}
}

// Flat ground grid (x,y in [-10,10], z=0, spacing 1) with a 3x3 hole at x,y in {-1,0,1} where
// only an elevated "building" patch (z=5, same footprint) sits instead -- the classic ground
// filter scenario where an object entirely occludes the ground beneath it, so the filter must
// rely on the morphological opening (not a per-cell min-Z shortcut) to detect it.
TEST(GroundExtractorTest, DetectsIsolatedElevatedObjectSurroundedByGround)
{
	GroundExtractor extractor;
	size_t idx = 0;
	size_t cornerIndex = 0, nearBuildingIndex = 0;
	bool haveCorner = false, haveNearBuilding = false;
	for (int x = -10; x <= 10; ++x) {
		for (int y = -10; y <= 10; ++y) {
			const bool insideHole = (x >= -1 && x <= 1 && y >= -1 && y <= 1);
			if (insideHole) continue;
			extractor.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
			if (x == -10 && y == -10 && !haveCorner) { cornerIndex = idx; haveCorner = true; }
			if (x == 2 && y == 0 && !haveNearBuilding) { nearBuildingIndex = idx; haveNearBuilding = true; }
			++idx;
		}
	}
	std::vector<size_t> buildingIndices;
	for (int x = -1; x <= 1; ++x) {
		for (int y = -1; y <= 1; ++y) {
			buildingIndices.push_back(idx);
			extractor.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 5.0f));
			++idx;
		}
	}

	GroundExtractor::Params params;
	params.cellSize = 1.0f;
	ASSERT_TRUE(extractor.extract(params));

	const auto flags = extractor.getGroundFlags();
	ASSERT_EQ(idx, flags.size());

	EXPECT_TRUE(flags[cornerIndex]);        // far ground corner: ground
	EXPECT_TRUE(flags[nearBuildingIndex]);  // ground right next to the building: still ground
	for (auto bi : buildingIndices) {
		EXPECT_FALSE(flags[bi]);            // elevated building points: non-ground
	}
}

// A ground point directly beneath a much taller point in the same grid cell: the cell's
// minimum-elevation grid value is dominated by the ground point, so the elevated point should
// be rejected purely by the final per-point height check even though its cell stays "ground".
TEST(GroundExtractorTest, ElevatedPointSharingGroundCellIsRejectedByHeightCheck)
{
	GroundExtractor extractor;
	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			extractor.add(Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
		}
	}
	// The elevated point shares the cell of (0,0).
	extractor.add(Vector3df(0.0f, 0.0f, 5.0f));

	GroundExtractor::Params params;
	params.cellSize = 1.0f;
	ASSERT_TRUE(extractor.extract(params));

	const auto flags = extractor.getGroundFlags();
	ASSERT_EQ(122u, flags.size());
	EXPECT_TRUE(flags[60]);   // the (0,0) ground point itself (index 60 = 11*5+5 in row-major x,y loop order)
	EXPECT_FALSE(flags.back()); // the elevated point appended last
}
