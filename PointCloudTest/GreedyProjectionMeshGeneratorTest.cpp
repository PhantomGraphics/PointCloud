#include "pch.h"

#include "../PointCloud/GreedyProjectionMeshGenerator.h"

#include <algorithm>
#include <set>
#include <sstream>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {
    std::string vertexKey(const Vector3df& v)
    {
        std::ostringstream oss;
        oss << std::round(v.x * 10000.0f) / 10000.0f << ","
            << std::round(v.y * 10000.0f) / 10000.0f << ","
            << std::round(v.z * 10000.0f) / 10000.0f;
        return oss.str();
    }

    std::string triangleKey(const Triangle3df& tri)
    {
        auto vs = tri.getVertices();
        std::array<std::string, 3> keys = { vertexKey(vs[0]), vertexKey(vs[1]), vertexKey(vs[2]) };
        std::sort(keys.begin(), keys.end());
        return keys[0] + "|" + keys[1] + "|" + keys[2];
    }
}

TEST(GreedyProjectionMeshGeneratorTest, EmptyInput)
{
    GreedyProjectionMeshGenerator generator;
    generator.generate({});
    EXPECT_TRUE(generator.getTriangles().empty());
}

TEST(GreedyProjectionMeshGeneratorTest, PlanarGrid)
{
    std::vector<Vector3df> points;
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            points.emplace_back(static_cast<float>(x) * 0.1f, static_cast<float>(y) * 0.1f, 0.0f);
        }
    }

    GreedyProjectionMeshGenerator generator;
    generator.generate(points);
    EXPECT_GT(generator.getTriangles().size(), 0u);
}

TEST(GreedyProjectionMeshGeneratorTest, NoDuplicateTriangles)
{
    std::vector<Vector3df> points;
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            points.emplace_back(static_cast<float>(x) * 0.1f, static_cast<float>(y) * 0.1f, 0.0f);
        }
    }

    GreedyProjectionMeshGenerator generator;
    generator.generate(points);
    const auto& tris = generator.getTriangles();

    std::set<std::string> unique;
    for (const auto& tri : tris) {
        unique.insert(triangleKey(tri));
    }

    EXPECT_EQ(unique.size(), tris.size());
}

TEST(GreedyProjectionMeshGeneratorTest, OutputConnectivity)
{
    std::vector<Vector3df> points;
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            points.emplace_back(static_cast<float>(x) * 0.2f, static_cast<float>(y) * 0.2f, 0.0f);
        }
    }

    GreedyProjectionMeshGenerator generator;
    generator.generate(points);
    const auto& tris = generator.getTriangles();
    ASSERT_GT(tris.size(), 1u);

    bool hasSharedEdge = false;
    for (size_t i = 0; i < tris.size() && !hasSharedEdge; ++i) {
        const auto vi = tris[i].getVertices();
        for (size_t j = i + 1; j < tris.size() && !hasSharedEdge; ++j) {
            const auto vj = tris[j].getVertices();
            int shared = 0;
            for (const auto& a : vi) {
                for (const auto& b : vj) {
                    if (getDistanceSquared(a, b) < 1e-10f) {
                        ++shared;
                    }
                }
            }
            if (shared >= 2) {
                hasSharedEdge = true;
            }
        }
    }

    EXPECT_TRUE(hasSharedEdge);
}
