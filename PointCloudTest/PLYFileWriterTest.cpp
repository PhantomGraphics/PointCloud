#include "pch.h"

#include "../PointCloud/PLYFileWriter.h"
#include "../PointCloud/PLYFileReader.h"
#include "../PointCloud/PLYFile.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>

using namespace Phantom;

// ---------------------------------------------------------------------------
// WriteAscii
// ---------------------------------------------------------------------------

TEST(PLYFileWriterTest, WriteAscii_HeaderContent)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_ascii_header.ply";

	EXPECT_TRUE(writer.writeAscii(filename, file));

	std::ifstream ifs(filename);
	ASSERT_TRUE(ifs.is_open());
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	EXPECT_NE(content.find("ply\n"),                   std::string::npos);
	EXPECT_NE(content.find("format ascii 1.0\n"),      std::string::npos);
	EXPECT_NE(content.find("element vertex 2\n"),      std::string::npos);
	EXPECT_NE(content.find("property float x\n"),      std::string::npos);
	EXPECT_NE(content.find("property float y\n"),      std::string::npos);
	EXPECT_NE(content.find("property float z\n"),      std::string::npos);
	EXPECT_NE(content.find("end_header\n"),            std::string::npos);

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, WriteAscii_PointValues)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_ascii_points.ply";

	EXPECT_TRUE(writer.writeAscii(filename, file));

	std::ifstream ifs(filename);
	ASSERT_TRUE(ifs.is_open());
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);
	for (const auto& p : file.getPoints()) {
		oss.str(""); oss.clear();
		oss << p.x << " " << p.y << " " << p.z;
		EXPECT_NE(content.find(oss.str()), std::string::npos) << "Expected line: " << oss.str();
	}

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, WriteAscii_Empty)
{
	PC::PLYFile file; // no points

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_ascii_empty.ply";

	EXPECT_TRUE(writer.writeAscii(filename, file));

	std::ifstream ifs(filename);
	ASSERT_TRUE(ifs.is_open());
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	EXPECT_NE(content.find("element vertex 0\n"), std::string::npos);
	EXPECT_NE(content.find("end_header\n"),        std::string::npos);

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, WriteAscii_InvalidPath)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));

	PC::PLYFileWriter writer;
	EXPECT_FALSE(writer.writeAscii("Z:/nonexistent/path/out.ply", file));
}

// ---------------------------------------------------------------------------
// WriteBinary
// ---------------------------------------------------------------------------

TEST(PLYFileWriterTest, WriteBinary_HeaderContent)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_binary_header.ply";

	EXPECT_TRUE(writer.writeBinary(filename, file));

	std::ifstream ifs(filename, std::ios::binary);
	ASSERT_TRUE(ifs.is_open());
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	EXPECT_NE(content.find("ply\n"),                              std::string::npos);
	EXPECT_NE(content.find("format binary_little_endian 1.0\n"), std::string::npos);
	EXPECT_NE(content.find("element vertex 2\n"),                 std::string::npos);
	EXPECT_NE(content.find("end_header\n"),                       std::string::npos);

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, WriteBinary_DataSize)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));
	file.getPoints().push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_binary_size.ply";

	EXPECT_TRUE(writer.writeBinary(filename, file));

	std::ifstream ifs(filename, std::ios::binary);
	ASSERT_TRUE(ifs.is_open());
	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	const std::string marker = "end_header\n";
	const auto pos = content.find(marker);
	ASSERT_NE(pos, std::string::npos);

	const size_t binarySize   = content.size() - (pos + marker.size());
	const size_t expectedSize = file.size() * 3 * sizeof(float);
	EXPECT_EQ(binarySize, expectedSize);

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, WriteBinary_InvalidPath)
{
	PC::PLYFile file;
	file.getPoints().push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));

	PC::PLYFileWriter writer;
	EXPECT_FALSE(writer.writeBinary("Z:/nonexistent/path/out.ply", file));
}

// ---------------------------------------------------------------------------
// Round-trip tests (writer 竊・reader)
// ---------------------------------------------------------------------------

TEST(PLYFileWriterTest, RoundTrip_Ascii)
{
	PC::PLYFile src;
	src.getPoints().push_back(Math::Vector3df(1.0f,  2.0f,  3.0f));
	src.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));
	src.getPoints().push_back(Math::Vector3df(0.0f,  0.0f,  0.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_roundtrip_ascii.ply";

	ASSERT_TRUE(writer.writeAscii(filename, src));

	PC::PLYFileReader reader;
	ASSERT_TRUE(reader.read(filename));

	const auto& pts = reader.getFile().getPoints();
	ASSERT_EQ(pts.size(), src.size());
	for (size_t i = 0; i < pts.size(); ++i) {
		EXPECT_NEAR(pts[i].x, src.getPoints()[i].x, 1e-5f);
		EXPECT_NEAR(pts[i].y, src.getPoints()[i].y, 1e-5f);
		EXPECT_NEAR(pts[i].z, src.getPoints()[i].z, 1e-5f);
	}

	std::remove(filename.c_str());
}

TEST(PLYFileWriterTest, RoundTrip_Binary)
{
	PC::PLYFile src;
	src.getPoints().push_back(Math::Vector3df(1.0f,  2.0f,  3.0f));
	src.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));
	src.getPoints().push_back(Math::Vector3df(0.0f,  0.0f,  0.0f));

	PC::PLYFileWriter writer;
	const std::string filename = "PLYFileWriterTest_roundtrip_binary.ply";

	ASSERT_TRUE(writer.writeBinary(filename, src));

	PC::PLYFileReader reader;
	ASSERT_TRUE(reader.read(filename));

	const auto& pts = reader.getFile().getPoints();
	ASSERT_EQ(pts.size(), src.size());
	for (size_t i = 0; i < pts.size(); ++i) {
		EXPECT_FLOAT_EQ(pts[i].x, src.getPoints()[i].x);
		EXPECT_FLOAT_EQ(pts[i].y, src.getPoints()[i].y);
		EXPECT_FLOAT_EQ(pts[i].z, src.getPoints()[i].z);
	}

	std::remove(filename.c_str());
}
