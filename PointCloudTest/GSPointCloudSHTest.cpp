#include "pch.h"

// Phase 3 of docs/todo/PLAN_gsview_gaussian_point_pbvr.md: standard-PLY loading
// of the f_rest_* spherical-harmonics coefficients.
#include "../PointCloud/GSPointCloud.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

using Phantom::PointCloud::GSPointCloud;

namespace {

// Minimal binary GS-PLY with a chosen number of f_rest_* properties.
void writeShPLY(const std::string& path, size_t n, int restProps)
{
    std::ofstream ofs(path, std::ios::binary);
    ASSERT_TRUE(ofs.is_open());
    ofs << "ply\nformat binary_little_endian 1.0\nelement vertex " << n << "\n";
    for (const char* p : { "x","y","z","f_dc_0","f_dc_1","f_dc_2" })
        ofs << "property float " << p << "\n";
    for (int i = 0; i < restProps; ++i)
        ofs << "property float f_rest_" << i << "\n";
    for (const char* p : { "opacity","scale_0","scale_1","scale_2","rot_0","rot_1","rot_2","rot_3" })
        ofs << "property float " << p << "\n";
    ofs << "end_header\n";

    const size_t stride = 6 + restProps + 8;
    for (size_t i = 0; i < n; ++i) {
        std::vector<float> v(stride, 0.0f);
        v[0] = float(i); v[1] = 0.0f; v[2] = 0.0f;
        v[3] = 0.1f; v[4] = 0.2f; v[5] = 0.3f;
        for (int k = 0; k < restProps; ++k)
            v[6 + k] = 0.01f * float(k) + 0.001f * float(i);  // distinct, recoverable
        v[6 + restProps + 0] = 2.0f; // opacity logit
        v[6 + restProps + 4] = 1.0f; // rot_0 = w
        ofs.write(reinterpret_cast<const char*>(v.data()),
                  static_cast<std::streamsize>(v.size() * sizeof(float)));
    }
}

} // namespace

TEST(GSPointCloudSH, DcOnlyPlyHasZeroDegreeAndNoRest)
{
    const std::string path = "gs_sh_dc.ply";
    writeShPLY(path, 4, 0);

    GSPointCloud c;
    ASSERT_TRUE(c.readFromFile(path));
    EXPECT_EQ(c.points.size(), 4u);
    EXPECT_EQ(c.shDegree, 0);
    EXPECT_TRUE(c.shRest.empty());
}

TEST(GSPointCloudSH, RestCountMapsToDegree)
{
    struct Case { int restProps; int degree; };
    for (Case cs : { Case{9, 1}, Case{24, 2}, Case{45, 3} }) {
        const std::string path = "gs_sh_deg.ply";
        writeShPLY(path, 3, cs.restProps);

        GSPointCloud c;
        ASSERT_TRUE(c.readFromFile(path)) << "restProps=" << cs.restProps;
        EXPECT_EQ(c.shDegree, cs.degree) << "restProps=" << cs.restProps;
        EXPECT_EQ(c.shRest.size(), static_cast<size_t>(3 * cs.restProps));
        EXPECT_EQ(GSPointCloud::coeffsPerChannel(cs.degree) * 3, cs.restProps);
    }
}

TEST(GSPointCloudSH, RestValuesAreReadInPropertyOrder)
{
    const std::string path = "gs_sh_vals.ply";
    writeShPLY(path, 2, 9); // degree 1

    GSPointCloud c;
    ASSERT_TRUE(c.readFromFile(path));
    ASSERT_EQ(c.shRest.size(), 18u);
    // point 0: f_rest_k = 0.01*k ; point 1: 0.01*k + 0.001
    for (int k = 0; k < 9; ++k) {
        EXPECT_NEAR(c.shRest[0 * 9 + k], 0.01f * k, 1e-5f) << "k=" << k;
        EXPECT_NEAR(c.shRest[1 * 9 + k], 0.01f * k + 0.001f, 1e-5f) << "k=" << k;
    }
}

TEST(GSPointCloudSH, NonStandardRestCountFallsBackToDcOnly)
{
    const std::string path = "gs_sh_weird.ply";
    writeShPLY(path, 2, 7); // not 9/24/45

    GSPointCloud c;
    ASSERT_TRUE(c.readFromFile(path));
    EXPECT_EQ(c.shDegree, 0);
    EXPECT_TRUE(c.shRest.empty());
}
