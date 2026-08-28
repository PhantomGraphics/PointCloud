#include "pch.h"

#include "../PointCloud/TXTFileReader.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	std::stringstream createStream()
	{
		std::stringstream stream;
		stream << "0.0 1.0 2.0\n";
		stream << "3.0 4.0 5.0\n";
		return stream;
	}
}

TEST(TXTFileReaderTest, TestRead)
{
	TXTFileReader reader;
	auto stream = createStream();
	reader.read(stream);
	const auto pos = reader.getPositions();
	EXPECT_EQ(2, pos.size());
}

TEST(TXTFileReaderTest, ReadValues)
{
	TXTFileReader reader;
	auto stream = createStream();
	reader.read(stream);
	const auto pos = reader.getPositions();
	ASSERT_EQ(2u, pos.size());
	EXPECT_FLOAT_EQ(0.0f, pos[0].x);
	EXPECT_FLOAT_EQ(1.0f, pos[0].y);
	EXPECT_FLOAT_EQ(2.0f, pos[0].z);
	EXPECT_FLOAT_EQ(3.0f, pos[1].x);
	EXPECT_FLOAT_EQ(4.0f, pos[1].y);
	EXPECT_FLOAT_EQ(5.0f, pos[1].z);
}

TEST(TXTFileReaderTest, EmptyStreamReturnsEmpty)
{
	TXTFileReader reader;
	std::stringstream emptyStream;
	reader.read(emptyStream);
	EXPECT_TRUE(reader.getPositions().empty());
}

TEST(TXTFileReaderTest, InvalidLineIncludesLineNumberInError)
{
	TXTFileReader reader;
	std::stringstream stream;
	stream << "0.0 1.0 2.0\n";
	stream << "a b c\n";

	EXPECT_FALSE(reader.read(stream));
	EXPECT_NE(reader.getLastError().find("line 2"), std::string::npos);
}
