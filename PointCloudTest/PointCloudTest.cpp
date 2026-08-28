#include "pch.h"

#include "../PointCloud/PointCloudFileLoader.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(PointCloudColoredDataTest, EmptySizeIsZero)
{
	PointCloudColoredData cloud;
	EXPECT_EQ(0u, cloud.size());
	EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudColoredDataTest, AddAndSize)
{
	PointCloudColoredData cloud;
	cloud.positions.push_back(Vector3df(0, 1, 2));

	ASSERT_EQ(1u, cloud.size());
	EXPECT_FALSE(cloud.empty());
	EXPECT_FLOAT_EQ(0.0f, cloud.positions[0].x);
	EXPECT_FLOAT_EQ(1.0f, cloud.positions[0].y);
	EXPECT_FLOAT_EQ(2.0f, cloud.positions[0].z);
}

TEST(PointCloudColoredDataTest, BoundingBoxFromMultiplePoints)
{
	PointCloudColoredData cloud;
	cloud.positions = {
		Vector3df(1.0f, 2.0f, 3.0f),
		Vector3df(-4.0f, 5.0f, 0.5f),
		Vector3df(2.5f, -1.0f, 10.0f),
	};

	const auto bb = cloud.getBoundingBox();
	const auto min = bb.getMin();
	const auto max = bb.getMax();

	EXPECT_FLOAT_EQ(-4.0f, min.x);
	EXPECT_FLOAT_EQ(-1.0f, min.y);
	EXPECT_FLOAT_EQ(0.5f, min.z);
	EXPECT_FLOAT_EQ(2.5f, max.x);
	EXPECT_FLOAT_EQ(5.0f, max.y);
	EXPECT_FLOAT_EQ(10.0f, max.z);
}

TEST(PointCloudColoredDataTest, HasColorsChecksSize)
{
	PointCloudColoredData cloud;
	cloud.positions = { Vector3df(0, 0, 0), Vector3df(1, 0, 0) };
	EXPECT_FALSE(cloud.hasColors());

	cloud.colors = { Vector3df(1, 0, 0), Vector3df(0, 1, 0) };
	EXPECT_TRUE(cloud.hasColors());
}

TEST(PointCloudColoredDataTest, HasNormalsChecksSize)
{
	PointCloudColoredData cloud;
	cloud.positions = { Vector3df(0, 0, 0), Vector3df(1, 0, 0) };
	EXPECT_FALSE(cloud.hasNormals());

	cloud.normals = { Vector3df(0, 0, 1), Vector3df(0, 0, 1) };
	EXPECT_TRUE(cloud.hasNormals());
}

TEST(PointCloudColoredDataTest, HasScalarsChecksSize)
{
	PointCloudColoredData cloud;
	cloud.positions = { Vector3df(0, 0, 0), Vector3df(1, 0, 0) };
	EXPECT_FALSE(cloud.hasScalars());

	cloud.scalars = { 0.5f, 1.5f };
	EXPECT_TRUE(cloud.hasScalars());
	EXPECT_FLOAT_EQ(0.5f, cloud.scalars[0]);
	EXPECT_FLOAT_EQ(1.5f, cloud.scalars[1]);
}

TEST(PointCloudColoredDataTest, ColorsAndNormalsDefaultEmpty)
{
	PointCloudColoredData cloud;
	EXPECT_TRUE(cloud.colors.empty());
	EXPECT_TRUE(cloud.normals.empty());
	EXPECT_TRUE(cloud.scalars.empty());
}
