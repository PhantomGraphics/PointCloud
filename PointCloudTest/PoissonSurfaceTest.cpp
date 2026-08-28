#include "pch.h"

#include "../PointCloud/PoissonSurface.h"

#include <random>

namespace {
    std::vector<PSPoint> createSpherePoints(size_t count, unsigned seed = 1234)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);

        std::vector<PSPoint> points;
        points.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            const float u = uni(rng);
            const float v = uni(rng);
            const float theta = 2.0f * 3.1415926535f * u;
            const float phi = std::acos(2.0f * v - 1.0f);
            const float x = std::sin(phi) * std::cos(theta);
            const float y = std::sin(phi) * std::sin(theta);
            const float z = std::cos(phi);
            points.push_back({ x, y, z, x, y, z });
        }
        return points;
    }

    std::vector<PSPoint> createPlanePoints(int width, int height)
    {
        std::vector<PSPoint> points;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const float px = (static_cast<float>(x) / static_cast<float>(width - 1)) - 0.5f;
                const float py = (static_cast<float>(y) / static_cast<float>(height - 1)) - 0.5f;
                points.push_back({ px, py, 0.0f, 0.0f, 0.0f, 1.0f });
            }
        }
        return points;
    }
}

TEST(PoissonSurfaceTest, EmptyInput)
{
    PoissonSurface surface;
    auto mesh = surface.reconstruct({});
    EXPECT_TRUE(mesh.vertices.empty());
    EXPECT_TRUE(mesh.faces.empty());
}

TEST(PoissonSurfaceTest, SphereSurface)
{
    PoissonSurface::Config cfg;
    cfg.resolution = 24;
    cfg.maxIters = 80;
    cfg.samplesPerCell = 1.2f;

    PoissonSurface surface(cfg);
    const auto points = createSpherePoints(300);
    auto mesh = surface.reconstruct(points);

    EXPECT_GT(mesh.vertices.size(), 0u);
    EXPECT_GT(mesh.faces.size(), 0u);
}

TEST(PoissonSurfaceTest, PlaneReconstruction)
{
    PoissonSurface::Config cfg;
    cfg.resolution = 20;
    cfg.maxIters = 60;
    cfg.samplesPerCell = 1.2f;

    PoissonSurface surface(cfg);
    const auto points = createPlanePoints(12, 12);
    auto mesh = surface.reconstruct(points);

    ASSERT_GT(mesh.vertices.size(), 0u);
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();
    for (const auto& v : mesh.vertices) {
        minZ = std::min(minZ, v[2]);
        maxZ = std::max(maxZ, v[2]);
    }
    EXPECT_LT(maxZ - minZ, 0.4f);
}

TEST(PoissonSurfaceTest, ResolutionParam)
{
    const auto points = createSpherePoints(220, 9876);

    PoissonSurface::Config lowCfg;
    lowCfg.resolution = 16;
    lowCfg.maxIters = 50;

    PoissonSurface::Config highCfg;
    highCfg.resolution = 28;
    highCfg.maxIters = 50;

    PoissonSurface low(lowCfg);
    PoissonSurface high(highCfg);

    auto lowMesh = low.reconstruct(points);
    auto highMesh = high.reconstruct(points);

    EXPECT_NE(lowMesh.faces.size(), highMesh.faces.size());
}
