#include "pch.h"

#include "../PointCloud/TXTFileReader.h"
#include "../PointCloud/TXTFileWriter.h"

#include <cstdio>
#include <sstream>

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(TXTFileWriterTest, WritesExpectedTextFormat)
{
	TXTFileWriter writer;
	writer.add(Vector3df(1.0f, 2.0f, 3.0f));
	writer.add(Vector3df(-4.5f, 0.25f, 10.0f));

	std::stringstream stream;
	ASSERT_TRUE(writer.write(stream));

	const auto content = stream.str();
	EXPECT_NE(content.find("1.000000 2.000000 3.000000\n"), std::string::npos);
	EXPECT_NE(content.find("-4.500000 0.250000 10.000000\n"), std::string::npos);
}

TEST(TXTFileWriterTest, WriteReadRoundTrip)
{
	TXTFileWriter writer;
	writer.add(Vector3df(0.0f, 1.0f, 2.0f));
	writer.add(Vector3df(3.0f, 4.0f, 5.0f));

	const std::string filename = "TXTFileWriterTest_roundtrip.txt";
	ASSERT_TRUE(writer.write(filename));

	TXTFileReader reader;
	ASSERT_TRUE(reader.read(filename));
	const auto pos = reader.getPositions();
	ASSERT_EQ(2u, pos.size());
	EXPECT_FLOAT_EQ(0.0f, pos[0].x);
	EXPECT_FLOAT_EQ(1.0f, pos[0].y);
	EXPECT_FLOAT_EQ(2.0f, pos[0].z);
	EXPECT_FLOAT_EQ(3.0f, pos[1].x);
	EXPECT_FLOAT_EQ(4.0f, pos[1].y);
	EXPECT_FLOAT_EQ(5.0f, pos[1].z);

	std::remove(filename.c_str());
}

TEST(TXTFileWriterTest, EmptyPointCloudWritesEmptyContent)
{
	TXTFileWriter writer;
	std::stringstream stream;

	ASSERT_TRUE(writer.write(stream));
	EXPECT_TRUE(stream.str().empty());
}
