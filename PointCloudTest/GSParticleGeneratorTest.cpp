#include "pch.h"

// CPU reference generator for the PBVR3DExperimental mode (see
// GSView/reference/GSParticleGenerator.h). Not on any render path; this test pins
// the particle-count formula and the determinism guarantee that Phase 1 of
// docs/todo/PLAN_gsview_gaussian_point_pbvr.md builds on.
#include "GSParticleGenerator.h"

#include <cmath>

using Phantom::PointCloud::GSPoint;
using Phantom::PointCloud::GSPointCloud;
using GSView::reference::GSParticleGenerator;

namespace {

GSPoint makeSplat(float opacityLogit)
{
    GSPoint p{};
    p.x = 1.0f; p.y = 2.0f; p.z = 3.0f;
    p.f_dc[0] = 0.2f; p.f_dc[1] = -0.1f; p.f_dc[2] = 0.4f;
    p.opacity = opacityLogit;
    p.scale[0] = std::log(0.05f);
    p.scale[1] = std::log(0.03f);
    p.scale[2] = std::log(0.02f);
    p.rot[0] = 1.0f; p.rot[1] = 0.0f; p.rot[2] = 0.0f; p.rot[3] = 0.0f;
    return p;
}

} // namespace

TEST(GSParticleGenerator, FullyTransparentSplatProducesNoParticles)
{
    GSPointCloud cloud;
    cloud.points.push_back(makeSplat(-40.0f)); // sigmoid(-40) ~= 0

    GSParticleGenerator gen;
    gen.setDensityScale(1.0f);
    gen.setMaxParticlesPerSplat(8);

    EXPECT_EQ(gen.particleCountForSplat(cloud.points[0]), 0);
    EXPECT_EQ(gen.generate(cloud).count(), 0u);
}

TEST(GSParticleGenerator, OpaqueSplatCountMatchesRoundedFormula)
{
    const GSPoint sp = makeSplat(1.0f); // sigmoid(1) ~= 0.7311
    GSParticleGenerator gen;
    gen.setDensityScale(1.0f);
    gen.setMaxParticlesPerSplat(8);

    // round(1.0 * 0.7311 * 8) = round(5.849) = 6
    EXPECT_EQ(gen.particleCountForSplat(sp), 6);
}

TEST(GSParticleGenerator, CountIsClampedToMaxParticlesPerSplat)
{
    const GSPoint sp = makeSplat(40.0f); // sigmoid ~= 1
    GSParticleGenerator gen;
    gen.setDensityScale(10.0f);          // would overshoot without the clamp
    gen.setMaxParticlesPerSplat(8);

    EXPECT_EQ(gen.particleCountForSplat(sp), 8);
}

TEST(GSParticleGenerator, GenerationIsDeterministicAcrossCalls)
{
    GSPointCloud cloud;
    cloud.points.push_back(makeSplat(2.0f));
    cloud.points.push_back(makeSplat(0.5f));

    GSParticleGenerator gen;
    gen.setDensityScale(1.5f);
    gen.setMaxParticlesPerSplat(8);
    gen.setSeed(1234u);

    const auto a = gen.generate(cloud);
    const auto b = gen.generate(cloud);

    ASSERT_EQ(a.count(), b.count());
    ASSERT_GT(a.count(), 0u);
    for (std::size_t i = 0; i < a.count(); ++i) {
        EXPECT_FLOAT_EQ(a.particles[i].pos.x, b.particles[i].pos.x);
        EXPECT_FLOAT_EQ(a.particles[i].pos.y, b.particles[i].pos.y);
        EXPECT_FLOAT_EQ(a.particles[i].pos.z, b.particles[i].pos.z);
        EXPECT_FLOAT_EQ(a.particles[i].color.r, b.particles[i].color.r);
    }
}

TEST(GSParticleGenerator, TotalParticleCountTracksDensityScale)
{
    GSPointCloud cloud;
    for (int i = 0; i < 20; ++i)
        cloud.points.push_back(makeSplat(0.0f)); // sigmoid(0) = 0.5

    GSParticleGenerator gen;
    gen.setMaxParticlesPerSplat(8);

    gen.setDensityScale(0.25f); // round(0.25 * 0.5 * 8) = round(1.0) = 1
    EXPECT_EQ(gen.generate(cloud).count(), 20u);

    gen.setDensityScale(1.0f);  // round(1.0 * 0.5 * 8) = 4
    EXPECT_EQ(gen.generate(cloud).count(), 80u);
}
