#include "pch.h"

#include "../PointCloud/GSPointCloud.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace Phantom::PointCloud;

namespace {

// ---------------------------------------------------------------------------
// Standard PLY helpers
// ---------------------------------------------------------------------------

void writeGSPLY(const std::string& filename, bool withNormals, size_t pointCount)
{
    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());

    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element vertex " << pointCount << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    if (withNormals) {
        ofs << "property float nx\n";
        ofs << "property float ny\n";
        ofs << "property float nz\n";
    }
    ofs << "property float f_dc_0\n";
    ofs << "property float f_dc_1\n";
    ofs << "property float f_dc_2\n";
    ofs << "property float opacity\n";
    ofs << "property float scale_0\n";
    ofs << "property float scale_1\n";
    ofs << "property float scale_2\n";
    ofs << "property float rot_0\n";
    ofs << "property float rot_1\n";
    ofs << "property float rot_2\n";
    ofs << "property float rot_3\n";
    ofs << "end_header\n";

    for (size_t i = 0; i < pointCount; ++i) {
        std::vector<float> values;
        values.push_back(static_cast<float>(i));
        values.push_back(static_cast<float>(i) + 0.1f);
        values.push_back(static_cast<float>(i) + 0.2f);
        if (withNormals) {
            values.push_back(0.0f);
            values.push_back(0.0f);
            values.push_back(1.0f);
        }
        values.push_back(0.2f);
        values.push_back(0.3f);
        values.push_back(0.4f);
        values.push_back(0.8f);
        values.push_back(1.0f);
        values.push_back(1.1f);
        values.push_back(1.2f);
        values.push_back(1.0f);
        values.push_back(0.0f);
        values.push_back(0.0f);
        values.push_back(0.0f);

        ofs.write(reinterpret_cast<const char*>(values.data()),
                  static_cast<std::streamsize>(values.size() * sizeof(float)));
    }
}

// ---------------------------------------------------------------------------
// Compressed PLY helpers
// ---------------------------------------------------------------------------

// Chunk data layout matches SuperSplat 2.x format (18 float properties):
//   min_x  min_y  min_z  max_x  max_y  max_z
//   min_scale_x  min_scale_y  min_scale_z
//   max_scale_x  max_scale_y  max_scale_z
//   min_r  min_g  min_b  max_r  max_g  max_b
struct TestChunk {
    float min_x, min_y, min_z, max_x, max_y, max_z;
    float min_scale_x, min_scale_y, min_scale_z;
    float max_scale_x, max_scale_y, max_scale_z;
    float min_r, min_g, min_b, max_r, max_g, max_b;
};
static_assert(sizeof(TestChunk) == 18 * sizeof(float), "TestChunk size mismatch");

struct TestVertex {
    uint32_t packed_position;
    uint32_t packed_rotation;
    uint32_t packed_scale;
    uint32_t packed_color;
};

// packed_rotation for approximately-identity quaternion (w≈1, x≈y≈z≈0):
//   idx=0, a=b=c=512 → a_unnorm ≈ 0.0007, m ≈ 1.0
static constexpr uint32_t kIdentityRot =
    (0u << 30) | (512u << 20) | (512u << 10) | 512u; // = 0x20080200

void writeCompressedPLY(const std::string& filename,
                        const std::vector<TestChunk>& chunks,
                        const std::vector<TestVertex>& vertices)
{
    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());

    const size_t nChunks   = chunks.size();
    const size_t nVertices = vertices.size();

    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "element chunk " << nChunks << "\n";
    ofs << "property float min_x\n";
    ofs << "property float min_y\n";
    ofs << "property float min_z\n";
    ofs << "property float max_x\n";
    ofs << "property float max_y\n";
    ofs << "property float max_z\n";
    ofs << "property float min_scale_x\n";
    ofs << "property float min_scale_y\n";
    ofs << "property float min_scale_z\n";
    ofs << "property float max_scale_x\n";
    ofs << "property float max_scale_y\n";
    ofs << "property float max_scale_z\n";
    ofs << "property float min_r\n";
    ofs << "property float min_g\n";
    ofs << "property float min_b\n";
    ofs << "property float max_r\n";
    ofs << "property float max_g\n";
    ofs << "property float max_b\n";
    ofs << "element vertex " << nVertices << "\n";
    ofs << "property uint packed_position\n";
    ofs << "property uint packed_rotation\n";
    ofs << "property uint packed_scale\n";
    ofs << "property uint packed_color\n";
    ofs << "element sh " << nVertices << "\n";
    for (int i = 0; i < 45; ++i)
        ofs << "property uchar f_rest_" << i << "\n";
    ofs << "end_header\n";

    ofs.write(reinterpret_cast<const char*>(chunks.data()),
              static_cast<std::streamsize>(nChunks * sizeof(TestChunk)));
    ofs.write(reinterpret_cast<const char*>(vertices.data()),
              static_cast<std::streamsize>(nVertices * sizeof(TestVertex)));

    // SH data: all zeros (45 bytes per vertex, not decoded)
    const std::vector<uint8_t> shZeros(nVertices * 45, 0);
    ofs.write(reinterpret_cast<const char*>(shZeros.data()),
              static_cast<std::streamsize>(shZeros.size()));
}

} // namespace

// ---------------------------------------------------------------------------
// Existing standard PLY tests
// ---------------------------------------------------------------------------

TEST(GSPointCloudTest, ReadValidFile)
{
    const std::string filename = "GSPointCloudTest_valid.ply";
    writeGSPLY(filename, true, 2);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(2u, cloud.points.size());
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].x);
    EXPECT_FLOAT_EQ(1.1f, cloud.points[1].y);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, MissingNormals)
{
    const std::string filename = "GSPointCloudTest_no_normals.ply";
    writeGSPLY(filename, false, 1);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(1u, cloud.points.size());
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].nx);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].ny);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].nz);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, SizeConsistency)
{
    const std::string filename = "GSPointCloudTest_size.ply";
    writeGSPLY(filename, true, 5);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    EXPECT_EQ(5u, cloud.points.size());

    std::remove(filename.c_str());
}

// ---------------------------------------------------------------------------
// Phase 1: robustness tests
// ---------------------------------------------------------------------------

TEST(GSPointCloudTest, ReadStandardPLY_CRLF)
{
    const std::string filename = "GSPointCloudTest_crlf.ply";
    {
        std::ofstream ofs(filename, std::ios::binary);
        ASSERT_TRUE(ofs.is_open());
        ofs << "ply\r\n";
        ofs << "format binary_little_endian 1.0\r\n";
        ofs << "element vertex 1\r\n";
        ofs << "property float x\r\n";
        ofs << "property float y\r\n";
        ofs << "property float z\r\n";
        ofs << "property float f_dc_0\r\n";
        ofs << "property float f_dc_1\r\n";
        ofs << "property float f_dc_2\r\n";
        ofs << "property float opacity\r\n";
        ofs << "property float scale_0\r\n";
        ofs << "property float scale_1\r\n";
        ofs << "property float scale_2\r\n";
        ofs << "property float rot_0\r\n";
        ofs << "property float rot_1\r\n";
        ofs << "property float rot_2\r\n";
        ofs << "property float rot_3\r\n";
        ofs << "end_header\r\n";

        float vals[] = { 1.0f, 2.0f, 3.0f,          // x y z
                         0.1f, 0.2f, 0.3f,           // f_dc
                         0.8f,                        // opacity
                         1.0f, 1.1f, 1.2f,           // scale
                         1.0f, 0.0f, 0.0f, 0.0f };   // rot
        ofs.write(reinterpret_cast<const char*>(vals),
                  static_cast<std::streamsize>(sizeof(vals)));
    }

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(1u, cloud.points.size());
    EXPECT_FLOAT_EQ(1.0f, cloud.points[0].x);
    EXPECT_FLOAT_EQ(2.0f, cloud.points[0].y);
    EXPECT_FLOAT_EQ(3.0f, cloud.points[0].z);
    EXPECT_FLOAT_EQ(0.8f, cloud.points[0].opacity);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadStandardPLY_MissingRequiredProperty)
{
    const std::string filename = "GSPointCloudTest_missing.ply";
    {
        std::ofstream ofs(filename, std::ios::binary);
        ASSERT_TRUE(ofs.is_open());
        ofs << "ply\n";
        ofs << "format binary_little_endian 1.0\n";
        ofs << "element vertex 1\n";
        ofs << "property float x\n";
        ofs << "property float y\n";
        ofs << "property float z\n";
        // opacity, f_dc_*, scale_*, rot_* are intentionally omitted
        ofs << "end_header\n";
        const float vals[] = { 1.0f, 2.0f, 3.0f };
        ofs.write(reinterpret_cast<const char*>(vals),
                  static_cast<std::streamsize>(sizeof(vals)));
    }

    GSPointCloud cloud;
    EXPECT_FALSE(cloud.readFromFile(filename));

    std::remove(filename.c_str());
}

// ---------------------------------------------------------------------------
// Phase 2: compressed PLY tests
// ---------------------------------------------------------------------------

TEST(GSPointCloudTest, ReadCompressedPLY_Basic)
{
    const std::string filename = "GSPointCloudTest_compressed_basic.ply";

    const TestChunk chunk0 = {
        0.0f, 0.0f, 0.0f, 4.0f, 2.0f, 6.0f,   // pos min/max
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,   // scale min/max
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f    // color min/max
    };

    // Vertex 0: all zeros → min-range values, opacity=-40
    // Vertex 1: all max → max-range values, opacity=40
    const std::vector<TestChunk>  chunks   = { chunk0 };
    const std::vector<TestVertex> vertices = {
        { 0u,          kIdentityRot, 0u,          0u          },
        { 0xFFFFFFFFu, kIdentityRot, 0xFFFFFFFFu, 0xFFFFFFFFu }
    };
    writeCompressedPLY(filename, chunks, vertices);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(2u, cloud.points.size());

    // Vertex 0: position = (min_x, min_y, min_z) = (0, 0, 0)
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].x);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].y);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].z);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].scale[0]);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].f_dc[0]);
    EXPECT_FLOAT_EQ(-40.0f, cloud.points[0].opacity);

    // Vertex 1: position = (max_x, max_y, max_z) = (4, 2, 6)
    EXPECT_FLOAT_EQ(4.0f, cloud.points[1].x);
    EXPECT_FLOAT_EQ(2.0f, cloud.points[1].y);
    EXPECT_FLOAT_EQ(6.0f, cloud.points[1].z);
    EXPECT_FLOAT_EQ(1.0f, cloud.points[1].scale[0]);
    EXPECT_FLOAT_EQ(1.0f, cloud.points[1].f_dc[0]);
    EXPECT_FLOAT_EQ(40.0f, cloud.points[1].opacity);

    // Rotation: approximately identity (w≈1, x≈y≈z≈0)
    EXPECT_NEAR(1.0f, cloud.points[0].rot[0], 0.002f);
    EXPECT_NEAR(0.0f, cloud.points[0].rot[1], 0.002f);
    EXPECT_NEAR(0.0f, cloud.points[0].rot[2], 0.002f);
    EXPECT_NEAR(0.0f, cloud.points[0].rot[3], 0.002f);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadCompressedPLY_OpacityLogit)
{
    const std::string filename = "GSPointCloudTest_compressed_opacity.ply";

    const TestChunk chunk0 = {
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f
    };

    // packed_color bits 0-7 carry the opacity byte
    // Vertex 0: op_byte=0   → op_norm=0.0   → opacity clamped to -40
    // Vertex 1: op_byte=255 → op_norm=1.0   → opacity clamped to +40
    // Vertex 2: op_byte=128 → op_norm=128/255 → opacity=log(128/127)
    const std::vector<TestChunk>  chunks   = { chunk0 };
    const std::vector<TestVertex> vertices = {
        { 0u, kIdentityRot, 0u, 0x00000000u },
        { 0u, kIdentityRot, 0u, 0x000000FFu },
        { 0u, kIdentityRot, 0u, 0x00000080u }
    };
    writeCompressedPLY(filename, chunks, vertices);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(3u, cloud.points.size());

    EXPECT_FLOAT_EQ(-40.0f, cloud.points[0].opacity);
    EXPECT_FLOAT_EQ( 40.0f, cloud.points[1].opacity);

    const float expectedMid = std::log(128.0f / 127.0f);
    EXPECT_NEAR(expectedMid, cloud.points[2].opacity, 1e-5f);

    std::remove(filename.c_str());
}

// ---------------------------------------------------------------------------
// .splat helpers
// ---------------------------------------------------------------------------

// .splat binary layout: 32 bytes per splat.
//   float pos[3], float scale[3] (already-exponentiated), uint8 color[4] (rgba), uint8 rot[4] (w,x,y,z)
struct RawSplatRecord {
    float   pos[3];
    float   scale[3];
    uint8_t color[4];
    uint8_t rot[4];
};
static_assert(sizeof(RawSplatRecord) == 32, "RawSplatRecord size mismatch");

void writeSplatFile(const std::string& filename, const std::vector<RawSplatRecord>& records)
{
    std::ofstream ofs(filename, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs.write(reinterpret_cast<const char*>(records.data()),
              static_cast<std::streamsize>(records.size() * sizeof(RawSplatRecord)));
}

// ---------------------------------------------------------------------------
// Phase 3: .splat tests
// ---------------------------------------------------------------------------

TEST(GSPointCloudTest, ReadSplat_Basic)
{
    const std::string filename = "GSPointCloudTest_basic.splat";

    const RawSplatRecord rec = {
        { 1.0f, 2.0f, 3.0f },
        { 0.1f, 0.2f, 0.3f },
        { 128, 64, 192, 200 },
        { 255, 128, 0, 64 }
    };
    writeSplatFile(filename, { rec });

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(1u, cloud.points.size());

    const auto& p = cloud.points[0];
    EXPECT_FLOAT_EQ(1.0f, p.x);
    EXPECT_FLOAT_EQ(2.0f, p.y);
    EXPECT_FLOAT_EQ(3.0f, p.z);

    EXPECT_NEAR(std::log(0.1f), p.scale[0], 1e-5f);
    EXPECT_NEAR(std::log(0.2f), p.scale[1], 1e-5f);
    EXPECT_NEAR(std::log(0.3f), p.scale[2], 1e-5f);

    constexpr float kShC0 = 0.28209479177387814f;
    EXPECT_NEAR((128.0f / 255.0f - 0.5f) / kShC0, p.f_dc[0], 1e-5f);
    EXPECT_NEAR(( 64.0f / 255.0f - 0.5f) / kShC0, p.f_dc[1], 1e-5f);
    EXPECT_NEAR((192.0f / 255.0f - 0.5f) / kShC0, p.f_dc[2], 1e-5f);

    const float a = 200.0f / 255.0f;
    EXPECT_NEAR(std::log(a / (1.0f - a)), p.opacity, 1e-5f);

    EXPECT_NEAR( 0.9921875f, p.rot[0], 1e-6f); // (255-128)/128
    EXPECT_NEAR( 0.0f,       p.rot[1], 1e-6f); // (128-128)/128
    EXPECT_NEAR(-1.0f,       p.rot[2], 1e-6f); // (0-128)/128
    EXPECT_NEAR(-0.5f,       p.rot[3], 1e-6f); // (64-128)/128

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadSplat_OpacityEdgeClamp)
{
    const std::string filename = "GSPointCloudTest_opacity_clamp.splat";

    const RawSplatRecord recMin = { {0,0,0}, {1,1,1}, {0,0,0,0},   {128,128,128,128} };
    const RawSplatRecord recMax = { {0,0,0}, {1,1,1}, {0,0,0,255}, {128,128,128,128} };
    writeSplatFile(filename, { recMin, recMax });

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(2u, cloud.points.size());
    EXPECT_FLOAT_EQ(-40.0f, cloud.points[0].opacity);
    EXPECT_FLOAT_EQ( 40.0f, cloud.points[1].opacity);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadSplat_InvalidFileSize)
{
    const std::string filename = "GSPointCloudTest_bad_size.splat";
    {
        std::ofstream ofs(filename, std::ios::binary);
        ASSERT_TRUE(ofs.is_open());
        const std::vector<uint8_t> bytes(20, 0); // not a multiple of 32
        ofs.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
    }

    GSPointCloud cloud;
    EXPECT_FALSE(cloud.readFromFile(filename));

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadSplat_MultipleRecords)
{
    const std::string filename = "GSPointCloudTest_multi.splat";

    std::vector<RawSplatRecord> records;
    for (int i = 0; i < 4; ++i) {
        records.push_back({
            { static_cast<float>(i), 0.0f, 0.0f },
            { 1.0f, 1.0f, 1.0f },
            { 128, 128, 128, 128 },
            { 128, 128, 128, 128 }
        });
    }
    writeSplatFile(filename, records);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(4u, cloud.points.size());
    for (int i = 0; i < 4; ++i)
        EXPECT_FLOAT_EQ(static_cast<float>(i), cloud.points[i].x);

    std::remove(filename.c_str());
}

TEST(GSPointCloudTest, ReadCompressedPLY_MultiChunk)
{
    const std::string filename = "GSPointCloudTest_compressed_multichunk.ply";

    // Chunk 0: x range [0, 1], Chunk 1: x range [10, 11]
    const TestChunk chunk0 = {
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f
    };
    const TestChunk chunk1 = {
        10.0f, 0.0f, 0.0f, 11.0f, 1.0f, 1.0f,
        0.0f,  0.0f, 0.0f,  1.0f, 1.0f, 1.0f,
        0.0f,  0.0f, 0.0f,  1.0f, 1.0f, 1.0f
    };

    // 512 vertices: 256 per chunk, all packed_position=0 → x=min_x
    std::vector<TestVertex> vertices(512, TestVertex{ 0u, kIdentityRot, 0u, 0u });

    writeCompressedPLY(filename, { chunk0, chunk1 }, vertices);

    GSPointCloud cloud;
    EXPECT_TRUE(cloud.readFromFile(filename));
    ASSERT_EQ(512u, cloud.points.size());

    // Vertices 0-255 belong to chunk 0: x = min_x = 0
    EXPECT_FLOAT_EQ(0.0f, cloud.points[0].x);
    EXPECT_FLOAT_EQ(0.0f, cloud.points[255].x);

    // Vertices 256-511 belong to chunk 1: x = min_x = 10
    EXPECT_FLOAT_EQ(10.0f, cloud.points[256].x);
    EXPECT_FLOAT_EQ(10.0f, cloud.points[511].x);

    std::remove(filename.c_str());
}
