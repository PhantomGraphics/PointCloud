#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/MLSSurface.h"
#include "../../CGLib/Math/Vector3d.h"
#include <cmath>
#include <functional>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

// 11x11 flat grid in the XY plane (x,y in [-5,5]), spacing 1, z given by zFunc(x,y).
// Index for (x,y) is (x+5)*11 + (y+5).
std::vector<Vector3df> makeGrid(const std::function<float(int, int)>& zFunc)
{
	std::vector<Vector3df> positions;
	for (int x = -5; x <= 5; ++x) {
		for (int y = -5; y <= 5; ++y) {
			positions.emplace_back(static_cast<float>(x), static_cast<float>(y), zFunc(x, y));
		}
	}
	return positions;
}

size_t gridIndex(int x, int y) { return static_cast<size_t>((x + 5) * 11 + (y + 5)); }

} // namespace

TEST(MLSSurfaceTest, EmptyInputProducesEmptyOutput)
{
	MLSSurface mls;
	mls.smooth(1.0);
	EXPECT_TRUE(mls.getSmoothedPoints().empty());
}

TEST(MLSSurfaceTest, OutputSizeMatchesInputSize)
{
	MLSSurface mls;
	mls.add(Vector3df(0, 0, 0));
	mls.add(Vector3df(1, 0, 0));
	mls.add(Vector3df(2, 0, 0));
	mls.smooth(1.5);
	EXPECT_EQ(3u, mls.getSmoothedPoints().size());
}

TEST(MLSSurfaceTest, TooFewNeighborsLeavesPositionUnchanged)
{
	// A single isolated point has zero neighbors -- not enough to build a PCA tangent frame.
	MLSSurface mls;
	mls.add(Vector3df(3, 4, 5));
	mls.smooth(1.0);
	const auto smoothed = mls.getSmoothedPoints();
	ASSERT_EQ(1u, smoothed.size());
	EXPECT_FLOAT_EQ(3.0f, smoothed[0].x);
	EXPECT_FLOAT_EQ(4.0f, smoothed[0].y);
	EXPECT_FLOAT_EQ(5.0f, smoothed[0].z);
}

TEST(MLSSurfaceTest, FlatGridStaysFlatAfterSmoothing)
{
	const auto positions = makeGrid([](int, int) { return 0.0f; });

	MLSSurface mls;
	for (const auto& p : positions) mls.add(p);
	mls.smooth(1.5);

	const auto smoothed = mls.getSmoothedPoints();
	ASSERT_EQ(positions.size(), smoothed.size());

	// An exactly flat height field fits with a zero constant term everywhere -- the fitted
	// surface passes through z=0, so smoothing should not move any point off the plane.
	const size_t centerIndex = gridIndex(0, 0);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].z, 1.0e-4f);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].x, 1.0e-4f);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].y, 1.0e-4f);
}

TEST(MLSSurfaceTest, NoisyCheckerboardGridIsSmoothedTowardZero)
{
	// z alternates +-0.5 in a checkerboard pattern -- the weighted quadric fit at any interior
	// point should pull the height back toward the local average, well below the raw amplitude.
	const auto positions = makeGrid([](int x, int y) { return ((x + y) % 2 == 0) ? 0.5f : -0.5f; });

	MLSSurface mls;
	for (const auto& p : positions) mls.add(p);
	mls.smooth(1.5);

	const auto smoothed = mls.getSmoothedPoints();
	ASSERT_EQ(positions.size(), smoothed.size());

	const size_t centerIndex = gridIndex(0, 0);
	EXPECT_LT(std::abs(smoothed[centerIndex].z), 0.5f);
}

TEST(MLSSurfaceTest, UpsampleOnEmptyInputReturnsEmpty)
{
	MLSSurface mls;
	const auto samples = mls.upsample(1.0, 0.5f, 0.25f);
	EXPECT_TRUE(samples.empty());
}

TEST(MLSSurfaceTest, UpsampleWithNonPositiveStepReturnsEmpty)
{
	MLSSurface mls;
	mls.add(Vector3df(0, 0, 0));
	mls.add(Vector3df(1, 0, 0));
	mls.add(Vector3df(0, 1, 0));
	const auto samples = mls.upsample(1.5, 0.5f, 0.0f);
	EXPECT_TRUE(samples.empty());
}

TEST(MLSSurfaceTest, UpsampleOnFlatGridProducesPointsNearZHeight)
{
	const auto positions = makeGrid([](int, int) { return 0.0f; });

	MLSSurface mls;
	for (const auto& p : positions) mls.add(p);

	const auto samples = mls.upsample(1.5, 0.5f, 0.25f);
	ASSERT_FALSE(samples.empty());
	for (const auto& s : samples) {
		EXPECT_NEAR(0.0f, s.z, 1.0e-4f);
	}
}

TEST(MLSSurfaceTest, SmoothKNNEmptyInputProducesEmptyOutput)
{
	MLSSurface mls;
	mls.smoothKNN(8);
	EXPECT_TRUE(mls.getSmoothedPoints().empty());
}

TEST(MLSSurfaceTest, SmoothKNNOutputSizeMatchesInputSize)
{
	MLSSurface mls;
	mls.add(Vector3df(0, 0, 0));
	mls.add(Vector3df(1, 0, 0));
	mls.add(Vector3df(2, 0, 0));
	mls.smoothKNN(2);
	EXPECT_EQ(3u, mls.getSmoothedPoints().size());
}

TEST(MLSSurfaceTest, SmoothKNNTooFewNeighborsLeavesPositionUnchanged)
{
	// A single isolated point has zero neighbors -- not enough to build a PCA tangent frame.
	MLSSurface mls;
	mls.add(Vector3df(3, 4, 5));
	mls.smoothKNN(8);
	const auto smoothed = mls.getSmoothedPoints();
	ASSERT_EQ(1u, smoothed.size());
	EXPECT_FLOAT_EQ(3.0f, smoothed[0].x);
	EXPECT_FLOAT_EQ(4.0f, smoothed[0].y);
	EXPECT_FLOAT_EQ(5.0f, smoothed[0].z);
}

TEST(MLSSurfaceTest, SmoothKNNFlatGridStaysFlatAfterSmoothing)
{
	const auto positions = makeGrid([](int, int) { return 0.0f; });

	MLSSurface mls;
	for (const auto& p : positions) mls.add(p);
	mls.smoothKNN(8);

	const auto smoothed = mls.getSmoothedPoints();
	ASSERT_EQ(positions.size(), smoothed.size());

	// An exactly flat height field fits with a zero constant term everywhere -- the fitted
	// surface passes through z=0, so smoothing should not move any point off the plane.
	const size_t centerIndex = gridIndex(0, 0);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].z, 1.0e-4f);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].x, 1.0e-4f);
	EXPECT_NEAR(0.0f, smoothed[centerIndex].y, 1.0e-4f);
}

TEST(MLSSurfaceTest, UpsampleKNNOnEmptyInputReturnsEmpty)
{
	MLSSurface mls;
	const auto samples = mls.upsampleKNN(8, 0.5f, 0.25f);
	EXPECT_TRUE(samples.empty());
}

TEST(MLSSurfaceTest, UpsampleKNNWithNonPositiveStepReturnsEmpty)
{
	MLSSurface mls;
	mls.add(Vector3df(0, 0, 0));
	mls.add(Vector3df(1, 0, 0));
	mls.add(Vector3df(0, 1, 0));
	const auto samples = mls.upsampleKNN(2, 0.5f, 0.0f);
	EXPECT_TRUE(samples.empty());
}

TEST(MLSSurfaceTest, UpsampleKNNOnFlatGridProducesPointsNearZHeight)
{
	const auto positions = makeGrid([](int, int) { return 0.0f; });

	MLSSurface mls;
	for (const auto& p : positions) mls.add(p);

	const auto samples = mls.upsampleKNN(8, 0.5f, 0.25f);
	ASSERT_FALSE(samples.empty());
	for (const auto& s : samples) {
		EXPECT_NEAR(0.0f, s.z, 1.0e-4f);
	}
}
