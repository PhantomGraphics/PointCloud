#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/DownSampler.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

// Empty input produces empty output
TEST(DownSamplerTest, EmptyInput)
{
	DownSampler sampler;
	sampler.execute(1.0);

	EXPECT_TRUE(sampler.getDownSampled().empty());
}

// A single point passes through unchanged
TEST(DownSamplerTest, SinglePoint)
{
	DownSampler sampler;
	sampler.add(Vector3df(1.0f, 2.0f, 3.0f));

	sampler.execute(1.0);

	const auto result = sampler.getDownSampled();
	ASSERT_EQ(1u, result.size());
	EXPECT_NEAR(1.0f, result[0][0], 1e-5f);
	EXPECT_NEAR(2.0f, result[0][1], 1e-5f);
	EXPECT_NEAR(3.0f, result[0][2], 1e-5f);
}

// Points farther apart than searchRadius are kept as separate output points
TEST(DownSamplerTest, PointsFarApartAreKeptSeparate)
{
	DownSampler sampler;
	sampler.add(Vector3df(0.0f, 0.0f, 0.0f));
	sampler.add(Vector3df(5.0f, 0.0f, 0.0f));  // distance 5.0 > searchRadius 1.0

	sampler.execute(1.0);

	EXPECT_EQ(2u, sampler.getDownSampled().size());
}

// Points within searchRadius are merged into a single output point
TEST(DownSamplerTest, ClosePointsAreMergedIntoOne)
{
	DownSampler sampler;
	sampler.add(Vector3df(0.0f, 0.0f, 0.0f));
	sampler.add(Vector3df(0.1f, 0.0f, 0.0f));
	sampler.add(Vector3df(0.2f, 0.0f, 0.0f));

	sampler.execute(1.0);

	EXPECT_EQ(1u, sampler.getDownSampled().size());
}

// The representative point is the centroid of the merged cluster
// (0,0,0) + (1,0,0) + (2,0,0) -> centroid (1,0,0)
TEST(DownSamplerTest, CentroidIsCorrect)
{
	DownSampler sampler;
	sampler.add(Vector3df(0.0f, 0.0f, 0.0f));
	sampler.add(Vector3df(1.0f, 0.0f, 0.0f));
	sampler.add(Vector3df(2.0f, 0.0f, 0.0f));

	sampler.execute(3.0);

	const auto result = sampler.getDownSampled();
	ASSERT_EQ(1u, result.size());
	EXPECT_NEAR(1.0f, result[0][0], 1e-5f);
	EXPECT_NEAR(0.0f, result[0][1], 1e-5f);
	EXPECT_NEAR(0.0f, result[0][2], 1e-5f);
}

// Output count must never exceed the input count
TEST(DownSamplerTest, OutputCountDoesNotExceedInput)
{
	DownSampler sampler;
	for (int i = 0; i < 10; ++i) {
		sampler.add(Vector3df(static_cast<float>(i) * 0.3f, 0.0f, 0.0f));
	}

	sampler.execute(1.0);

	EXPECT_LE(sampler.getDownSampled().size(), 10u);
}

// Two spatially separate clusters each produce one output point
TEST(DownSamplerTest, TwoSeparateGroups)
{
	DownSampler sampler;
	// Group A: x ~= 0
	sampler.add(Vector3df( 0.0f, 0.0f, 0.0f));
	sampler.add(Vector3df( 0.2f, 0.0f, 0.0f));
	// Group B: x ~= 10
	sampler.add(Vector3df(10.0f, 0.0f, 0.0f));
	sampler.add(Vector3df(10.2f, 0.0f, 0.0f));

	sampler.execute(1.0);

	ASSERT_EQ(2u, sampler.getDownSampled().size());
}

// This is the key property that distinguishes true voxel-grid downsampling (PCL's VoxelGrid /
// Open3D's voxel_down_sample -- what execute() is documented to implement) from a radius-based
// greedy clustering: voxel boundaries are fixed to the global grid, not centered on whichever
// point happens to be visited first. Two points essentially touching (distance 0.002) but on
// opposite sides of a cell boundary (x=0.999 -> floor(0.999/1.0)=0, x=1.001 -> floor(1.001/1.0)=1)
// must land in different voxels and stay separate, even though they'd certainly be merged by a
// search-radius-1.0 neighbor query centered on either point.
TEST(DownSamplerTest, PointsStraddlingAVoxelBoundaryStaySeparate)
{
	DownSampler sampler;
	sampler.add(Vector3df(0.999f, 0.0f, 0.0f)); // voxel x-index 0
	sampler.add(Vector3df(1.001f, 0.0f, 0.0f)); // voxel x-index 1

	sampler.execute(1.0);

	EXPECT_EQ(2u, sampler.getDownSampled().size());
}

// Voxel bucketing must floor towards negative infinity (not truncate towards zero), matching the
// standard voxel-grid convention: -0.1 and -0.9 both belong to voxel index -1 (the half-open cell
// [-1, 0)), while -1.1 belongs to the next voxel over (index -2).
TEST(DownSamplerTest, NegativeCoordinatesBucketByFlooring)
{
	DownSampler sampler;
	sampler.add(Vector3df(-0.1f, 0.0f, 0.0f)); // voxel x-index -1
	sampler.add(Vector3df(-0.9f, 0.0f, 0.0f)); // voxel x-index -1 (same cell as above)
	sampler.add(Vector3df(-1.1f, 0.0f, 0.0f)); // voxel x-index -2 (different cell)

	sampler.execute(1.0);

	ASSERT_EQ(2u, sampler.getDownSampled().size());
}
