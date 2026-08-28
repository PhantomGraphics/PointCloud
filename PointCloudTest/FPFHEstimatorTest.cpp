#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/FPFHEstimator.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

// 5x5 flat grid in the XY plane, spacing 1, all normals +Z, optionally shifted by `offset`.
std::pair<std::vector<Vector3df>, std::vector<Vector3df>> makeFlatGrid(const Vector3df& offset)
{
	std::vector<Vector3df> positions;
	std::vector<Vector3df> normals;
	for (int x = -2; x <= 2; ++x) {
		for (int y = -2; y <= 2; ++y) {
			positions.push_back(offset + Vector3df(static_cast<float>(x), static_cast<float>(y), 0.0f));
			normals.emplace_back(0.0f, 0.0f, 1.0f);
		}
	}
	return { positions, normals };
}

// Index of the grid's center point (x=0, y=0) for the loop order used by makeFlatGrid().
constexpr size_t kGridCenterIndex = 12;

} // namespace

TEST(FPFHEstimatorTest, FailsWithEmptyInput)
{
	FPFHEstimator estimator;
	EXPECT_FALSE(estimator.estimate(5));
}

TEST(FPFHEstimatorTest, FailsWithKZero)
{
	FPFHEstimator estimator;
	estimator.add(Vector3df(0, 0, 0), Vector3df(0, 0, 1));
	estimator.add(Vector3df(1, 0, 0), Vector3df(0, 0, 1));
	EXPECT_FALSE(estimator.estimate(0));
}

TEST(FPFHEstimatorTest, OutputSizeMatchesInputSize)
{
	const auto [positions, normals] = makeFlatGrid(Vector3df(0, 0, 0));

	FPFHEstimator estimator;
	for (size_t i = 0; i < positions.size(); ++i) estimator.add(positions[i], normals[i]);

	ASSERT_TRUE(estimator.estimate(8));
	EXPECT_EQ(positions.size(), estimator.getHistograms().size());
}

// The FPFH descriptor depends only on relative geometry (positions/normals fed in), so
// translating the whole cloud must leave every point's histogram unchanged.
TEST(FPFHEstimatorTest, TranslationInvariance)
{
	const auto [posA, normA] = makeFlatGrid(Vector3df(0.0f, 0.0f, 0.0f));
	const auto [posB, normB] = makeFlatGrid(Vector3df(50.0f, -30.0f, 20.0f));

	FPFHEstimator estA, estB;
	for (size_t i = 0; i < posA.size(); ++i) estA.add(posA[i], normA[i]);
	for (size_t i = 0; i < posB.size(); ++i) estB.add(posB[i], normB[i]);

	ASSERT_TRUE(estA.estimate(8));
	ASSERT_TRUE(estB.estimate(8));

	const auto histA = estA.getHistograms();
	const auto histB = estB.getHistograms();
	ASSERT_EQ(histA.size(), histB.size());

	const auto& hA = histA[kGridCenterIndex];
	const auto& hB = histB[kGridCenterIndex];
	for (int i = 0; i < FPFHEstimator::HistogramSize; ++i) {
		EXPECT_NEAR(hA[i], hB[i], 1.0e-4f);
	}
}

// A neighborhood with uniform normals (flat patch) should produce a clearly different
// histogram than an otherwise-identical neighborhood whose normals vary sharply from point to
// point (simulating a corner/rough patch), since the alpha/theta features are sensitive to
// normal-to-normal angle.
TEST(FPFHEstimatorTest, VaryingNeighborNormalsChangesHistogram)
{
	const auto [positions, uniformNormals] = makeFlatGrid(Vector3df(0, 0, 0));

	std::vector<Vector3df> checkerNormals;
	checkerNormals.reserve(positions.size());
	for (int x = -2; x <= 2; ++x) {
		for (int y = -2; y <= 2; ++y) {
			const bool flip = ((x + y) % 2 != 0);
			checkerNormals.push_back(flip ? Vector3df(1.0f, 0.0f, 0.0f) : Vector3df(0.0f, 0.0f, 1.0f));
		}
	}

	FPFHEstimator flatEst, checkerEst;
	for (size_t i = 0; i < positions.size(); ++i) {
		flatEst.add(positions[i], uniformNormals[i]);
		checkerEst.add(positions[i], checkerNormals[i]);
	}
	ASSERT_TRUE(flatEst.estimate(8));
	ASSERT_TRUE(checkerEst.estimate(8));

	const auto flatHist = flatEst.getHistograms()[kGridCenterIndex];
	const auto checkerHist = checkerEst.getHistograms()[kGridCenterIndex];

	float sqDist = 0.0f;
	for (int i = 0; i < FPFHEstimator::HistogramSize; ++i) {
		const float diff = flatHist[i] - checkerHist[i];
		sqDist += diff * diff;
	}
	EXPECT_GT(sqDist, 0.01f);
}
