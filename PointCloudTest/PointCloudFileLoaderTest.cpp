#include "pch.h"
#include "../PointCloud/PointCloudFileLoader.h"

#include <cstdio>
#include <fstream>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {

PointCloudColoredData makeSampleData()
{
	PointCloudColoredData data;
	data.positions = {
		Vector3df(1.0f, 2.0f, 3.0f),
		Vector3df(-4.5f, 0.25f, 10.0f),
	};
	data.colors = {
		Vector3df(0.25f, 0.5f, 0.75f),
		Vector3df(1.0f, 0.0f, 0.5f),
	};
	return data;
}

} // namespace

TEST(PointCloudFileLoaderTest, TxtRoundTripPreservesPositionsAndColors)
{
	const std::string filename = "PointCloudFileLoaderTest_roundtrip.txt";
	const auto original = makeSampleData();

	std::string err;
	ASSERT_TRUE(savePointCloud(filename, original, err));

	PointCloudColoredData loaded;
	ASSERT_TRUE(loadPointCloud(filename, loaded, err));
	ASSERT_EQ(original.size(), loaded.size());
	for (size_t i = 0; i < original.size(); ++i) {
		EXPECT_FLOAT_EQ(original.positions[i].x, loaded.positions[i].x);
		EXPECT_FLOAT_EQ(original.positions[i].y, loaded.positions[i].y);
		EXPECT_FLOAT_EQ(original.positions[i].z, loaded.positions[i].z);
		EXPECT_FLOAT_EQ(original.colors[i].x, loaded.colors[i].x);
		EXPECT_FLOAT_EQ(original.colors[i].y, loaded.colors[i].y);
		EXPECT_FLOAT_EQ(original.colors[i].z, loaded.colors[i].z);
	}

	std::remove(filename.c_str());
}

TEST(PointCloudFileLoaderTest, TxtWithoutColorColumnsUsesDefaultColor)
{
	const std::string filename = "PointCloudFileLoaderTest_no_color.txt";
	{
		std::ofstream ofs(filename);
		ASSERT_TRUE(ofs.is_open());
		ofs << "# comment line, should be skipped\n";
		ofs << "1.0 2.0 3.0\n";
	}

	PointCloudColoredData loaded;
	std::string err;
	ASSERT_TRUE(loadPointCloud(filename, loaded, err));
	ASSERT_EQ(1u, loaded.size());
	EXPECT_FLOAT_EQ(1.0f, loaded.positions[0].x);
	EXPECT_FLOAT_EQ(2.0f, loaded.positions[0].y);
	EXPECT_FLOAT_EQ(3.0f, loaded.positions[0].z);
	// No r/g/b columns present -> falls back to the loader's default color.
	EXPECT_FLOAT_EQ(0.6f, loaded.colors[0].x);
	EXPECT_FLOAT_EQ(0.8f, loaded.colors[0].y);
	EXPECT_FLOAT_EQ(1.0f, loaded.colors[0].z);

	std::remove(filename.c_str());
}

TEST(PointCloudFileLoaderTest, PcdRoundTripPreservesPositionsAndColors)
{
	const std::string filename = "PointCloudFileLoaderTest_roundtrip.pcd";
	const auto original = makeSampleData();

	std::string err;
	ASSERT_TRUE(savePointCloud(filename, original, err));

	PointCloudColoredData loaded;
	ASSERT_TRUE(loadPointCloud(filename, loaded, err));
	ASSERT_EQ(original.size(), loaded.size());
	for (size_t i = 0; i < original.size(); ++i) {
		EXPECT_FLOAT_EQ(original.positions[i].x, loaded.positions[i].x);
		EXPECT_FLOAT_EQ(original.positions[i].y, loaded.positions[i].y);
		EXPECT_FLOAT_EQ(original.positions[i].z, loaded.positions[i].z);
		EXPECT_NEAR(original.colors[i].x, loaded.colors[i].x, 1e-5f);
		EXPECT_NEAR(original.colors[i].y, loaded.colors[i].y, 1e-5f);
		EXPECT_NEAR(original.colors[i].z, loaded.colors[i].z, 1e-5f);
	}

	std::remove(filename.c_str());
}

// PLY round-trip only preserves positions: loadPointCloud's PLY path reads
// geometry via PLYFileReader and always assigns the loader's default color,
// it does not parse the red/green/blue properties written by savePointCloud.
TEST(PointCloudFileLoaderTest, PlyRoundTripPreservesPositionsOnly)
{
	const std::string filename = "PointCloudFileLoaderTest_roundtrip.ply";
	const auto original = makeSampleData();

	std::string err;
	ASSERT_TRUE(savePointCloud(filename, original, err));

	PointCloudColoredData loaded;
	ASSERT_TRUE(loadPointCloud(filename, loaded, err));
	ASSERT_EQ(original.size(), loaded.size());
	for (size_t i = 0; i < original.size(); ++i) {
		EXPECT_FLOAT_EQ(original.positions[i].x, loaded.positions[i].x);
		EXPECT_FLOAT_EQ(original.positions[i].y, loaded.positions[i].y);
		EXPECT_FLOAT_EQ(original.positions[i].z, loaded.positions[i].z);
		EXPECT_FLOAT_EQ(0.6f, loaded.colors[i].x);
		EXPECT_FLOAT_EQ(0.8f, loaded.colors[i].y);
		EXPECT_FLOAT_EQ(1.0f, loaded.colors[i].z);
	}

	std::remove(filename.c_str());
}

TEST(PointCloudFileLoaderTest, LoadFailsWithUnsupportedExtension)
{
	PointCloudColoredData loaded;
	std::string err;
	EXPECT_FALSE(loadPointCloud("PointCloudFileLoaderTest_unsupported.obj", loaded, err));
	EXPECT_FALSE(err.empty());
}

TEST(PointCloudFileLoaderTest, SaveFailsWithUnsupportedExtension)
{
	const auto data = makeSampleData();
	std::string err;
	EXPECT_FALSE(savePointCloud("PointCloudFileLoaderTest_unsupported.obj", data, err));
	EXPECT_FALSE(err.empty());
}

TEST(PointCloudFileLoaderTest, LoadFailsWhenFileDoesNotExist)
{
	PointCloudColoredData loaded;
	std::string err;
	EXPECT_FALSE(loadPointCloud("PointCloudFileLoaderTest_does_not_exist.pcd", loaded, err));
	EXPECT_FALSE(err.empty());
}

TEST(PointCloudFileLoaderTest, NormalsFieldDefaultsEmptyAndHasNormalsChecksSize)
{
	PointCloudColoredData data;
	EXPECT_TRUE(data.normals.empty());
	EXPECT_FALSE(data.hasNormals());

	data.positions = { Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f) };
	EXPECT_FALSE(data.hasNormals()); // size mismatch: normals still empty

	data.normals = { Vector3df(0.0f, 1.0f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f) };
	EXPECT_TRUE(data.hasNormals());
}

TEST(PointCloudFileLoaderTest, TxtRoundTripLeavesNormalsEmpty)
{
	const std::string filename = "PointCloudFileLoaderTest_normals_roundtrip.txt";
	const auto original = makeSampleData();

	std::string err;
	ASSERT_TRUE(savePointCloud(filename, original, err));

	PointCloudColoredData loaded;
	ASSERT_TRUE(loadPointCloud(filename, loaded, err));
	EXPECT_TRUE(loaded.normals.empty());
	EXPECT_FALSE(loaded.hasNormals());

	std::remove(filename.c_str());
}
