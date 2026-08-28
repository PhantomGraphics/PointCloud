#include "pch.h"

#include "../PointCloud/PCDFileWriter.h"
#include "../PointCloud/PCDFile.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>

using namespace Phantom;

TEST(PCDFileWriterTest, WriteAscii)
{
	PC::PCDFile file;
	// 点を追加
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));

	PC::PCDFileWriter writer;
	const std::string filename = "PCDFileWriterTest_ascii.pcd";

	EXPECT_TRUE(writer.writeAscii(filename, file));

	// ファイルを読み込んでヘッダと座標表記を確認
	std::ifstream ifs(filename);
	ASSERT_TRUE(ifs.is_open());

	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
	EXPECT_NE(content.find("FIELDS x y z"), std::string::npos);
	EXPECT_NE(content.find("DATA ascii"), std::string::npos);

	// 各点が6桁固定小数で出力されているか確認
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);
	for (const auto& p : file.getPoints()) {
		oss.str("");
		oss.clear();
		oss << p.x << " " << p.y << " " << p.z;
		EXPECT_NE(content.find(oss.str()), std::string::npos) << "Expected line: " << oss.str();
	}

	ifs.close();
	std::remove(filename.c_str());
}

TEST(PCDFileWriterTest, WriteBinary)
{
	PC::PCDFile file;
	// 点を追加
	file.getPoints().push_back(Math::Vector3df(1.0f, 2.0f, 3.0f));
	file.getPoints().push_back(Math::Vector3df(-4.5f, 0.25f, 10.0f));
	file.getPoints().push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));

	PC::PCDFileWriter writer;
	const std::string filename = "PCDFileWriterTest_binary.pcd";

	EXPECT_TRUE(writer.writeBinary(filename, file));

	// バイナリファイルを読み込み、ヘッダの直後にあるバイナリデータのサイズを確認
	std::ifstream ifs(filename, std::ios::binary);
	ASSERT_TRUE(ifs.is_open());

	std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
	const std::string marker = "DATA binary\n";
	auto pos = content.find(marker);
	ASSERT_NE(pos, std::string::npos);

	size_t headerEnd = pos + marker.size();
	size_t binarySize = content.size() - headerEnd;

	const size_t expectedBinarySize = file.size() * 3 * sizeof(float); // x,y,z each float
	EXPECT_EQ(binarySize, expectedBinarySize);

	ifs.close();
	std::remove(filename.c_str());
}

