#include "pch.h"

#include "../PointCloud/PCDFileReader.h"
#include "../PointCloud/PCDFileWriter.h"
#include "../PointCloud/PCDFile.h"

#include <fstream>
#include <cstdio>

using namespace Phantom;

TEST(PCDFileReaderTest, ReadAscii)
{
	const std::string filename = "PCDFileReaderTest_ascii.pcd";

	std::ofstream ofs(filename);
	ASSERT_TRUE(ofs.is_open());
	ofs << "VERSION .7\n";
	ofs << "FIELDS x y z\n";
	ofs << "SIZE 4 4 4\n";
	ofs << "TYPE F F F\n";
	ofs << "COUNT 1 1 1\n";
	ofs << "WIDTH 2\n";
	ofs << "HEIGHT 1\n";
	ofs << "POINTS 2\n";
	ofs << "DATA ascii\n";
	ofs << "1.0 2.0 3.0\n";
	ofs << "-4.5 0.25 10.0\n";
	ofs.close();

	PC::PCDFileReader reader;
	EXPECT_TRUE(reader.read(filename));

	const auto& points = reader.getFile().getPoints();
	ASSERT_EQ(points.size(), 2u);
	EXPECT_FLOAT_EQ(points[0].x, 1.0f);
	EXPECT_FLOAT_EQ(points[0].y, 2.0f);
	EXPECT_FLOAT_EQ(points[0].z, 3.0f);
	EXPECT_FLOAT_EQ(points[1].x, -4.5f);
	EXPECT_FLOAT_EQ(points[1].y, 0.25f);
	EXPECT_FLOAT_EQ(points[1].z, 10.0f);

	std::remove(filename.c_str());
}

TEST(PCDFileReaderTest, ReadBinary)
{
	PC::PCDFile file;
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));
	file.getPoints().push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));

	const std::string filename = "PCDFileReaderTest_binary.pcd";
	PC::PCDFileWriter writer;
	ASSERT_TRUE(writer.writeBinary(filename, file));

	PC::PCDFileReader reader;
	EXPECT_TRUE(reader.read(filename));

	const auto& points = reader.getFile().getPoints();
	ASSERT_EQ(points.size(), file.size());
	for (size_t i = 0; i < points.size(); ++i) {
		EXPECT_FLOAT_EQ(points[i].x, file.getPoints()[i].x);
		EXPECT_FLOAT_EQ(points[i].y, file.getPoints()[i].y);
		EXPECT_FLOAT_EQ(points[i].z, file.getPoints()[i].z);
	}

	std::remove(filename.c_str());
}

TEST(PCDFileReaderTest, ReadFailsOnNonNumericPointsHeaderInsteadOfCrashing)
{
	// Previously an unguarded std::stoul on the "POINTS" header value would
	// throw std::invalid_argument here; it must now fail gracefully instead.
	const std::string filename = "PCDFileReaderTest_bad_points.pcd";

	std::ofstream ofs(filename);
	ASSERT_TRUE(ofs.is_open());
	ofs << "VERSION .7\n";
	ofs << "FIELDS x y z\n";
	ofs << "SIZE 4 4 4\n";
	ofs << "TYPE F F F\n";
	ofs << "COUNT 1 1 1\n";
	ofs << "WIDTH 1\n";
	ofs << "HEIGHT 1\n";
	ofs << "POINTS not_a_number\n";
	ofs << "DATA ascii\n";
	ofs << "1.0 2.0 3.0\n";
	ofs.close();

	PC::PCDFileReader reader;
	EXPECT_FALSE(reader.read(filename));
	EXPECT_FALSE(reader.getLastError().empty());

	std::remove(filename.c_str());
}

TEST(PCDFileReaderTest, ReadFailsOnNonNumericSizeHeaderInsteadOfCrashing)
{
	// Previously an unguarded std::stoi on the "SIZE" header values would
	// throw std::invalid_argument here; it must now fail gracefully instead.
	const std::string filename = "PCDFileReaderTest_bad_size.pcd";

	std::ofstream ofs(filename);
	ASSERT_TRUE(ofs.is_open());
	ofs << "VERSION .7\n";
	ofs << "FIELDS x y z\n";
	ofs << "SIZE 4 not_a_number 4\n";
	ofs << "TYPE F F F\n";
	ofs << "COUNT 1 1 1\n";
	ofs << "WIDTH 1\n";
	ofs << "HEIGHT 1\n";
	ofs << "POINTS 1\n";
	ofs << "DATA ascii\n";
	ofs << "1.0 2.0 3.0\n";
	ofs.close();

	PC::PCDFileReader reader;
	EXPECT_FALSE(reader.read(filename));
	EXPECT_FALSE(reader.getLastError().empty());

	std::remove(filename.c_str());
}

TEST(PCDFileReaderTest, ReadFailsWhenMissingXYZField)
{
	const std::string filename = "PCDFileReaderTest_missing_xyz.pcd";

	std::ofstream ofs(filename);
	ASSERT_TRUE(ofs.is_open());
	ofs << "VERSION .7\n";
	ofs << "FIELDS x y\n";
	ofs << "SIZE 4 4\n";
	ofs << "TYPE F F\n";
	ofs << "COUNT 1 1\n";
	ofs << "WIDTH 1\n";
	ofs << "HEIGHT 1\n";
	ofs << "POINTS 1\n";
	ofs << "DATA ascii\n";
	ofs << "1.0 2.0\n";
	ofs.close();

	PC::PCDFileReader reader;
	EXPECT_FALSE(reader.read(filename));
	EXPECT_FALSE(reader.getLastError().empty());

	std::remove(filename.c_str());
}
