#include "pch.h"

#include "../Core/PLYFileReader.h"
#include "../Core/PLYFileWriter.h"
#include "../Core/PointSet.h"
#include "../Core/PointSetFile.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace Phantom;
using namespace Phantom::PC;

namespace {

	PointSet makeSet(size_t n, bool withNormals = false)
	{
		PointSet s;
		for (size_t i = 0; i < n; ++i) {
			const float f = static_cast<float>(i);
			s.positions.emplace_back(f, f * 0.5f, -f);
			s.colors.emplace_back(0.1f + 0.01f * f, 0.5f, 0.25f);
			s.radii.push_back(0.01f + 0.001f * f);
			if (withNormals) s.normals.emplace_back(0.0f, 1.0f, 0.0f);
		}
		return s;
	}

	std::filesystem::path tempPath(const char* name)
	{
		return std::filesystem::temp_directory_path() / name;
	}

	void writeText(const std::filesystem::path& p, const std::string& text)
	{
		std::ofstream ofs(p, std::ios::binary);
		ofs << text;
	}

}

TEST(PointSetTest, ValidateAcceptsConsistentSet)
{
	EXPECT_TRUE(makeSet(5, true).validate().empty());
	EXPECT_TRUE(PointSet{}.validate().empty());   // an empty point cloud is allowed
}

TEST(PointSetTest, ValidateRejectsBadAttributes)
{
	auto s = makeSet(3);
	s.colors.pop_back();
	EXPECT_NE(std::string::npos, s.validate().find("color count"));

	s = makeSet(3);
	s.radii[1] = 0.0f;
	EXPECT_NE(std::string::npos, s.validate().find("point 1"));

	s = makeSet(3);
	s.radii[2] = std::numeric_limits<float>::quiet_NaN();
	EXPECT_NE(std::string::npos, s.validate().find("point 2"));

	s = makeSet(3);
	s.positions[0].x = std::numeric_limits<float>::infinity();
	EXPECT_NE(std::string::npos, s.validate().find("non-finite coordinate"));

	s = makeSet(3, true);
	s.normals.pop_back();
	EXPECT_NE(std::string::npos, s.validate().find("normal count"));
}

TEST(PointSetTest, MakeSharedRejectsInvalid)
{
	std::string error;
	auto bad = makeSet(2);
	bad.radii[0] = -1.0f;
	EXPECT_EQ(nullptr, makeSharedPointSet(std::move(bad), &error));
	EXPECT_FALSE(error.empty());

	auto shared = makeSharedPointSet(makeSet(2), &error);
	ASSERT_NE(nullptr, shared);
	EXPECT_EQ(2u, shared->size());
}

TEST(PointSetTest, BoundsWithAndWithoutRadius)
{
	PointSet s;
	s.positions = { { 0, 0, 0 }, { 2, 4, 6 } };
	s.colors = { { 1, 1, 1 }, { 1, 1, 1 } };
	s.radii = { 0.5f, 1.0f };
	const auto b = s.bounds();
	EXPECT_FLOAT_EQ(0.0f, b.getMin().x);
	EXPECT_FLOAT_EQ(6.0f, b.getMax().z);
	const auto r = s.boundsWithRadius();
	EXPECT_FLOAT_EQ(-0.5f, r.getMin().x);
	EXPECT_FLOAT_EQ(3.0f, r.getMax().x);
	EXPECT_FLOAT_EQ(7.0f, r.getMax().z);
}

TEST(PointSetTest, SubsetKeepsOrderAndAttributes)
{
	const auto s = makeSet(5, true);
	const auto sub = subset(s, { 3, 1, 3 });
	ASSERT_TRUE(sub.has_value());
	ASSERT_EQ(3u, sub->size());
	EXPECT_FLOAT_EQ(3.0f, sub->positions[0].x);
	EXPECT_FLOAT_EQ(1.0f, sub->positions[1].x);
	EXPECT_FLOAT_EQ(s.radii[3], sub->radii[2]);
	EXPECT_EQ(3u, sub->normals.size());
	EXPECT_FALSE(subset(s, { 5 }).has_value());
}

TEST(PointSetTest, DefaultRadiusScalesWithCloudSize)
{
	PointSet s;
	for (int i = 0; i < 1000; ++i) s.positions.emplace_back(static_cast<float>(i % 10), static_cast<float>((i / 10) % 10), static_cast<float>(i / 100));
	const float r = defaultRadiusForBounds(s.bounds(), s.size());
	EXPECT_GT(r, 0.3f);
	EXPECT_LT(r, 0.7f);   // about half the 1.0 grid spacing... of a 9-unit cube with 1000 points
	EXPECT_GT(defaultRadiusForBounds(Math::Box3df::createDegeneratedBox(), 0), 0.0f);
	// A single point has no extent; the radius must still be usable.
	PointSet one;
	one.positions = { { 1, 2, 3 } };
	EXPECT_GT(defaultRadiusForBounds(one.bounds(), 1), 0.0f);
	// A flat cloud must not collapse the radius to the thickness.
	PointSet flat;
	for (int i = 0; i < 100; ++i) flat.positions.emplace_back(static_cast<float>(i % 10), static_cast<float>(i / 10), 0.0f);
	EXPECT_GT(defaultRadiusForBounds(flat.bounds(), flat.size()), 0.05f);
}

TEST(PointSetTest, ContentHashDetectsChanges)
{
	auto a = makeSet(4, true);
	auto b = a;
	EXPECT_EQ(contentHash(a), contentHash(b));
	b.colors[2].y += 0.001f;
	EXPECT_NE(contentHash(a), contentHash(b));
}

TEST(PointSetTest, SrgbRoundTrip)
{
	for (float v = 0.0f; v <= 1.0f; v += 0.05f)
		EXPECT_NEAR(v, linearToSrgb(srgbToLinear(v)), 1.0e-5f);
	EXPECT_NEAR(0.2140f, srgbToLinear(0.5f), 1.0e-3f);
}

TEST(PointSetFileTest, BinaryRoundTripIsExact)
{
	const auto path = tempPath("PointSetFileTest_roundtrip.pset");
	const auto src = makeSet(1000, true);
	std::string error;
	ASSERT_TRUE(savePointSetBinary(path, src, error)) << error;
	EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));

	PointSet back;
	uint64_t hash = 0;
	ASSERT_TRUE(loadPointSetBinary(path, back, hash, error)) << error;
	EXPECT_EQ(contentHash(src), hash);
	ASSERT_EQ(src.size(), back.size());
	EXPECT_EQ(src.positions, back.positions);
	EXPECT_EQ(src.colors, back.colors);
	EXPECT_EQ(src.radii, back.radii);
	EXPECT_EQ(src.normals, back.normals);
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, BinaryEmptyAndNoNormals)
{
	const auto path = tempPath("PointSetFileTest_empty.pset");
	std::string error;
	ASSERT_TRUE(savePointSetBinary(path, PointSet{}, error)) << error;
	PointSet back = makeSet(3);
	uint64_t hash = 0;
	ASSERT_TRUE(loadPointSetBinary(path, back, hash, error)) << error;
	EXPECT_TRUE(back.empty());

	ASSERT_TRUE(savePointSetBinary(path, makeSet(7), error)) << error;
	ASSERT_TRUE(loadPointSetBinary(path, back, hash, error)) << error;
	EXPECT_EQ(7u, back.size());
	EXPECT_FALSE(back.hasNormals());
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, BinaryRejectsMissingTruncatedAndCorrupt)
{
	const auto path = tempPath("PointSetFileTest_corrupt.pset");
	std::string error;
	PointSet out;
	uint64_t hash = 0;

	std::filesystem::remove(path);
	EXPECT_FALSE(loadPointSetBinary(path, out, hash, error));
	EXPECT_NE(std::string::npos, error.find("not found"));
	EXPECT_NE(std::string::npos, error.find("PointSetFileTest_corrupt"));

	ASSERT_TRUE(savePointSetBinary(path, makeSet(50), error)) << error;
	const auto size = std::filesystem::file_size(path);

	// Truncated: size no longer matches the declared count.
	std::filesystem::resize_file(path, size - 5);
	EXPECT_FALSE(loadPointSetBinary(path, out, hash, error));
	EXPECT_NE(std::string::npos, error.find("truncated"));

	// Corrupt payload byte, correct size: the content hash catches it.
	ASSERT_TRUE(savePointSetBinary(path, makeSet(50), error)) << error;
	{
		std::fstream f(path, std::ios::in | std::ios::out | std::ios::binary);
		f.seekp(40);
		const char b = 0x55;
		f.write(&b, 1);
	}
	EXPECT_FALSE(loadPointSetBinary(path, out, hash, error));
	EXPECT_NE(std::string::npos, error.find("hash"));

	// Not a point set at all.
	writeText(path, std::string(64, 'x'));
	EXPECT_FALSE(loadPointSetBinary(path, out, hash, error));
	EXPECT_NE(std::string::npos, error.find("not a point set"));

	// A newer format version is refused with its version number.
	ASSERT_TRUE(savePointSetBinary(path, makeSet(2), error)) << error;
	{
		std::fstream f(path, std::ios::in | std::ios::out | std::ios::binary);
		f.seekp(4);
		const uint32_t v = kPointSetBinaryVersion + 1;
		f.write(reinterpret_cast<const char*>(&v), 4);
	}
	EXPECT_FALSE(loadPointSetBinary(path, out, hash, error));
	EXPECT_NE(std::string::npos, error.find("version"));
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, PlyRoundTripBinaryAndAscii)
{
	for (const bool binary : { true, false }) {
		const auto path = tempPath(binary ? "PointSetFileTest_rt_bin.ply" : "PointSetFileTest_rt_ascii.ply");
		auto src = makeSet(20, true);
		// Colours that are exact 8-bit sRGB values survive the linear <-> sRGB trip.
		for (size_t i = 0; i < src.size(); ++i)
			src.colors[i] = { srgbToLinear(static_cast<float>(i * 10) / 255.0f), srgbToLinear(0.5f), srgbToLinear(1.0f) };
		std::string error;
		ASSERT_TRUE(savePointSetToPly(path, src, binary, error)) << error;

		PointSet back;
		PointSetImportInfo info;
		ASSERT_TRUE(loadPointSetFromPly(path, {}, back, info, error)) << error;
		EXPECT_TRUE(info.hadColor);
		EXPECT_TRUE(info.hadRadius);
		EXPECT_TRUE(info.hadNormals);
		ASSERT_EQ(src.size(), back.size());
		for (size_t i = 0; i < src.size(); ++i) {
			EXPECT_NEAR(src.positions[i].x, back.positions[i].x, 1.0e-5f);
			EXPECT_NEAR(src.radii[i], back.radii[i], 1.0e-5f);
			EXPECT_NEAR(src.colors[i].x, back.colors[i].x, 2.0e-3f);
			EXPECT_NEAR(src.colors[i].z, back.colors[i].z, 2.0e-3f);
			EXPECT_NEAR(1.0f, back.normals[i].y, 1.0e-5f);
		}
		std::filesystem::remove(path);
	}
}

TEST(PointSetFileTest, PlyWithoutColorOrRadiusGetsDefaults)
{
	const auto path = tempPath("PointSetFileTest_defaults.ply");
	writeText(path, "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
		"0 0 0\n1 0 0\n0 1 0\n");
	PointSetImportOptions opt;
	opt.defaultColor = { 0.25f, 0.5f, 0.75f };
	PointSet out;
	PointSetImportInfo info;
	std::string error;
	ASSERT_TRUE(loadPointSetFromPly(path, opt, out, info, error)) << error;
	EXPECT_FALSE(info.hadColor);
	EXPECT_FALSE(info.hadRadius);
	EXPECT_GT(info.appliedRadius, 0.0f);
	EXPECT_EQ(opt.defaultColor, out.colors[1]);
	EXPECT_FLOAT_EQ(info.appliedRadius, out.radii[2]);

	opt.defaultRadius = 0.125f;
	ASSERT_TRUE(loadPointSetFromPly(path, opt, out, info, error)) << error;
	EXPECT_FLOAT_EQ(0.125f, out.radii[0]);
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, PlyUcharColorIsTreatedAsSrgb)
{
	const auto path = tempPath("PointSetFileTest_srgb.ply");
	writeText(path, "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\n"
		"property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n0 0 0 128 255 0\n");
	PointSet out;
	PointSetImportInfo info;
	std::string error;
	ASSERT_TRUE(loadPointSetFromPly(path, {}, out, info, error)) << error;
	EXPECT_NEAR(srgbToLinear(128.0f / 255.0f), out.colors[0].x, 1.0e-5f);
	EXPECT_NEAR(1.0f, out.colors[0].y, 1.0e-5f);
	EXPECT_NEAR(0.0f, out.colors[0].z, 1.0e-5f);
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, PlyRejectsInvalidAndLeavesOutputUntouched)
{
	const auto path = tempPath("PointSetFileTest_invalid.ply");
	const std::string head = "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\n";
	PointSet out = makeSet(4);
	PointSetImportInfo info;
	std::string error;

	writeText(path, head + "end_header\n0 0 0\nnan 0 0\n");
	EXPECT_FALSE(loadPointSetFromPly(path, {}, out, info, error));
	EXPECT_NE(std::string::npos, error.find("non-finite coordinate"));
	EXPECT_EQ(4u, out.size());

	writeText(path, head + "property float radius\nend_header\n0 0 0 0.1\n1 0 0 -0.1\n");
	EXPECT_FALSE(loadPointSetFromPly(path, {}, out, info, error));
	EXPECT_NE(std::string::npos, error.find("invalid radius"));
	EXPECT_EQ(4u, out.size());

	writeText(path, head + "end_header\n0 0 0\n");   // fewer vertices than declared
	EXPECT_FALSE(loadPointSetFromPly(path, {}, out, info, error));
	EXPECT_FALSE(error.empty());

	EXPECT_FALSE(loadPointSetFromPly(tempPath("PointSetFileTest_no_such_file.ply"), {}, out, info, error));
	EXPECT_NE(std::string::npos, error.find("PointSetFileTest_no_such_file"));
	std::filesystem::remove(path);
}

TEST(PointSetFileTest, PlyEmptyVertexListIsValid)
{
	const auto path = tempPath("PointSetFileTest_emptyply.ply");
	writeText(path, "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
	PointSet out = makeSet(2);
	PointSetImportInfo info;
	std::string error;
	ASSERT_TRUE(loadPointSetFromPly(path, {}, out, info, error)) << error;
	EXPECT_TRUE(out.empty());
	std::filesystem::remove(path);
}

TEST(PLYFileReaderRadiusTest, ReadsRadiusFromBinaryAndAscii)
{
	const auto path = tempPath("PLYFileReaderRadiusTest.ply");
	PLYFile file;
	file.getPoints() = { { 1, 2, 3 }, { 4, 5, 6 } };
	file.getRadii() = { 0.25f, 0.5f };
	PLYFileWriter writer;
	ASSERT_TRUE(writer.writeBinary(path, file));
	PLYFileReader reader;
	ASSERT_TRUE(reader.read(path));
	ASSERT_TRUE(reader.getFile().hasRadii());
	EXPECT_FLOAT_EQ(0.5f, reader.getFile().getRadii()[1]);
	EXPECT_FALSE(reader.getFile().hasColors());

	ASSERT_TRUE(writer.writeAscii(path, file));
	ASSERT_TRUE(reader.read(path));
	ASSERT_TRUE(reader.getFile().hasRadii());
	EXPECT_FLOAT_EQ(0.25f, reader.getFile().getRadii()[0]);
	std::filesystem::remove(path);
}

TEST(PLYFileWriterAttributesTest, PositionOnlyOutputIsUnchanged)
{
	const auto path = tempPath("PLYFileWriterAttributesTest.ply");
	PLYFile file;
	file.getPoints() = { { 1, 2, 3 } };
	PLYFileWriter writer;
	ASSERT_TRUE(writer.writeAscii(path, file));
	std::ifstream ifs(path);
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
	EXPECT_EQ(std::string::npos, content.find("red"));
	EXPECT_EQ(std::string::npos, content.find("radius"));
	EXPECT_EQ(std::string::npos, content.find("nx"));
	ifs.close();
	std::filesystem::remove(path);
}
