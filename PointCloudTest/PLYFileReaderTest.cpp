#include "pch.h"

#include "../PointCloud/PLYFileReader.h"

#include <fstream>
#include <cstdio>
#include <cstdint>
#include <cstring>

using namespace Phantom::PC;

namespace {

void writeFloatBig(std::ofstream& ofs, float v)
{
    uint32_t u = 0;
    std::memcpy(&u, &v, sizeof(u));
    const uint32_t be = ((u & 0xFF000000u) >> 24)
        | ((u & 0x00FF0000u) >> 8)
        | ((u & 0x0000FF00u) << 8)
        | ((u & 0x000000FFu) << 24);
    ofs.write(reinterpret_cast<const char*>(&be), sizeof(be));
}

}

TEST(PLYFileReaderTest, ReadAscii)
{
    const std::string filename = "PLYFileReaderTest_ascii.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex 2\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    ofs << "1.0 2.0 3.0\n";
    ofs << "-4.0 5.5 6.0\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_TRUE(reader.getLastError().empty());
    ASSERT_EQ(2u, reader.getFile().size());
    EXPECT_FLOAT_EQ(1.0f, reader.getFile().getPoints()[0].x);
    EXPECT_FLOAT_EQ(2.0f, reader.getFile().getPoints()[0].y);
    EXPECT_FLOAT_EQ(3.0f, reader.getFile().getPoints()[0].z);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadBinaryLittleFloat32)
{
    const std::string filename = "PLYFileReaderTest_binary_le_f32.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex 2\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";

    const float p0[3] = { 1.0f, 2.5f, -3.0f };
    const float p1[3] = { -4.25f, 5.0f, 6.75f };
    ofs.write(reinterpret_cast<const char*>(p0), sizeof(p0));
    ofs.write(reinterpret_cast<const char*>(p1), sizeof(p1));
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_TRUE(reader.getLastError().empty());
    ASSERT_EQ(2u, reader.getFile().size());
    EXPECT_FLOAT_EQ(1.0f, reader.getFile().getPoints()[0].x);
    EXPECT_FLOAT_EQ(2.5f, reader.getFile().getPoints()[0].y);
    EXPECT_FLOAT_EQ(-3.0f, reader.getFile().getPoints()[0].z);
    EXPECT_FLOAT_EQ(-4.25f, reader.getFile().getPoints()[1].x);
    EXPECT_FLOAT_EQ(5.0f, reader.getFile().getPoints()[1].y);
    EXPECT_FLOAT_EQ(6.75f, reader.getFile().getPoints()[1].z);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadBinaryBigFloat32)
{
    const std::string filename = "PLYFileReaderTest_binary_be_f32.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_big_endian 1.0\n";
    ofs << "element vertex 2\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";

    writeFloatBig(ofs, 1.0f);
    writeFloatBig(ofs, -2.0f);
    writeFloatBig(ofs, 3.5f);
    writeFloatBig(ofs, -4.0f);
    writeFloatBig(ofs, 5.25f);
    writeFloatBig(ofs, 6.0f);
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_TRUE(reader.getLastError().empty());
    ASSERT_EQ(2u, reader.getFile().size());
    EXPECT_FLOAT_EQ(1.0f, reader.getFile().getPoints()[0].x);
    EXPECT_FLOAT_EQ(-2.0f, reader.getFile().getPoints()[0].y);
    EXPECT_FLOAT_EQ(3.5f, reader.getFile().getPoints()[0].z);
    EXPECT_FLOAT_EQ(-4.0f, reader.getFile().getPoints()[1].x);
    EXPECT_FLOAT_EQ(5.25f, reader.getFile().getPoints()[1].y);
    EXPECT_FLOAT_EQ(6.0f, reader.getFile().getPoints()[1].z);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadBinaryLittleFloat64)
{
    const std::string filename = "PLYFileReaderTest_binary_le_f64.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex 1\n";
    ofs << "property double x\n";
    ofs << "property double y\n";
    ofs << "property double z\n";
    ofs << "end_header\n";

    const double p0[3] = { 1.25, -2.5, 3.75 };
    ofs.write(reinterpret_cast<const char*>(p0), sizeof(p0));
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_TRUE(reader.getLastError().empty());
    ASSERT_EQ(1u, reader.getFile().size());
    EXPECT_FLOAT_EQ(1.25f, reader.getFile().getPoints()[0].x);
    EXPECT_FLOAT_EQ(-2.5f, reader.getFile().getPoints()[0].y);
    EXPECT_FLOAT_EQ(3.75f, reader.getFile().getPoints()[0].z);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadBinaryLittleIntegerProperties)
{
    const std::string filename = "PLYFileReaderTest_binary_le_int.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex 1\n";
    ofs << "property int x\n";
    ofs << "property uchar y\n";
    ofs << "property short z\n";
    ofs << "end_header\n";

    const int32_t x = -100;
    const uint8_t y = 200;
    const int16_t z = 300;
    ofs.write(reinterpret_cast<const char*>(&x), sizeof(x));
    ofs.write(reinterpret_cast<const char*>(&y), sizeof(y));
    ofs.write(reinterpret_cast<const char*>(&z), sizeof(z));
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_TRUE(reader.getLastError().empty());
    ASSERT_EQ(1u, reader.getFile().size());
    EXPECT_FLOAT_EQ(-100.0f, reader.getFile().getPoints()[0].x);
    EXPECT_FLOAT_EQ(200.0f, reader.getFile().getPoints()[0].y);
    EXPECT_FLOAT_EQ(300.0f, reader.getFile().getPoints()[0].z);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadAsciiWithColorsAndNormals)
{
    const std::string filename = "PLYFileReaderTest_ascii_rgbn.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex 2\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "property float nx\n";
    ofs << "property float ny\n";
    ofs << "property float nz\n";
    ofs << "end_header\n";
    ofs << "1.0 2.0 3.0 255 128 0 0.0 0.0 1.0\n";
    ofs << "4.0 5.0 6.0 0 51 204 1.0 0.0 0.0\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    const auto& file = reader.getFile();
    ASSERT_EQ(2u, file.size());
    ASSERT_TRUE(file.hasColors());
    ASSERT_TRUE(file.hasNormals());
    ASSERT_EQ(2u, file.getColors().size());
    ASSERT_EQ(2u, file.getNormals().size());
    EXPECT_FLOAT_EQ(1.0f, file.getColors()[0].x);
    EXPECT_NEAR(128.0f / 255.0f, file.getColors()[0].y, 1.0e-6f);
    EXPECT_FLOAT_EQ(0.0f, file.getColors()[0].z);
    EXPECT_NEAR(204.0f / 255.0f, file.getColors()[1].z, 1.0e-6f);
    EXPECT_FLOAT_EQ(1.0f, file.getNormals()[0].z);
    EXPECT_FLOAT_EQ(1.0f, file.getNormals()[1].x);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadBinaryLittleWithColorsAndNormals)
{
    const std::string filename = "PLYFileReaderTest_binary_le_rgbn.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex 2\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "property float nx\n";
    ofs << "property float ny\n";
    ofs << "property float nz\n";
    ofs << "property float curvature\n";
    ofs << "end_header\n";

    // Two rows laid out exactly as declared above. curvature is an extra scalar the
    // reader must skip without disturbing the color/normal byte offsets, which is how
    // real scanner output (PCL fragment exports) is laid out.
    const float pos[2][3] = { { 1.0f, 2.0f, 3.0f }, { -4.0f, 5.5f, 6.0f } };
    const uint8_t rgb[2][3] = { { 255, 128, 0 }, { 0, 51, 204 } };
    const float nrm[2][3] = { { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } };
    for (int i = 0; i < 2; ++i) {
        ofs.write(reinterpret_cast<const char*>(pos[i]), sizeof(pos[i]));
        ofs.write(reinterpret_cast<const char*>(rgb[i]), sizeof(rgb[i]));
        ofs.write(reinterpret_cast<const char*>(nrm[i]), sizeof(nrm[i]));
        const float curvature = 0.25f;
        ofs.write(reinterpret_cast<const char*>(&curvature), sizeof(curvature));
    }
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    const auto& file = reader.getFile();
    ASSERT_EQ(2u, file.size());
    ASSERT_TRUE(file.hasColors());
    ASSERT_TRUE(file.hasNormals());
    EXPECT_FLOAT_EQ(-4.0f, file.getPoints()[1].x);
    EXPECT_FLOAT_EQ(1.0f, file.getColors()[0].x);
    EXPECT_NEAR(128.0f / 255.0f, file.getColors()[0].y, 1.0e-6f);
    EXPECT_NEAR(51.0f / 255.0f, file.getColors()[1].y, 1.0e-6f);
    EXPECT_FLOAT_EQ(1.0f, file.getNormals()[0].z);
    EXPECT_FLOAT_EQ(1.0f, file.getNormals()[1].x);

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadWithoutColorPropertiesLeavesColorsEmpty)
{
    const std::string filename = "PLYFileReaderTest_ascii_nocolor.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex 1\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    ofs << "1.0 2.0 3.0\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_TRUE(reader.read(filename));
    EXPECT_FALSE(reader.getFile().hasColors());
    EXPECT_FALSE(reader.getFile().hasNormals());
    EXPECT_TRUE(reader.getFile().getColors().empty());

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadFailsWhenVertexHasNoProperties)
{
    const std::string filename = "PLYFileReaderTest_no_props.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex 1\n";
    ofs << "end_header\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_FALSE(reader.read(filename));
    EXPECT_NE(std::string::npos, reader.getLastError().find("x/y/z"));

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadFailsAndStoresError)
{
    const std::string filename = "PLYFileReaderTest_invalid.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "not_ply\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_FALSE(reader.read(filename));
    EXPECT_FALSE(reader.getLastError().empty());

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadFailsOnNonNumericElementCountInsteadOfCrashing)
{
    // Previously an unguarded std::stoul on "element vertex <count>" would
    // throw std::invalid_argument here; it must now fail gracefully instead.
    const std::string filename = "PLYFileReaderTest_bad_count.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex not_a_number\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    ofs << "1.0 2.0 3.0\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_FALSE(reader.read(filename));
    EXPECT_FALSE(reader.getLastError().empty());

    std::remove(filename.c_str());
}

TEST(PLYFileReaderTest, ReadFailsOnNonNumericAsciiVertexInsteadOfCrashing)
{
    // Previously an unguarded std::stof on the ASCII vertex line would throw
    // std::invalid_argument here; it must now fail gracefully instead.
    const std::string filename = "PLYFileReaderTest_bad_vertex.ply";

    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex 1\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    ofs << "1.0 not_a_number 3.0\n";
    ofs.close();

    PLYFileReader reader;
    EXPECT_FALSE(reader.read(filename));
    EXPECT_FALSE(reader.getLastError().empty());

    std::remove(filename.c_str());
}
