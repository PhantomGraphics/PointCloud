#include "pch.h"

// Phase 6 of docs/todo/PLAN_gsview_gaussian_point_pbvr.md: verify the GPU
// algorithm against the analytic oracle at the algorithm level. A CPU port of
// the GPU pipeline (renderMonteCarlo) must converge to the analytic ground truth
// (renderAnalytic) -- PSNR >= 35 dB, SSIM >= 0.98 -- which is the deferred
// Phase 3 completion criterion.
#include "GaussianPointOracle.h"

#include <glm/gtc/quaternion.hpp>

#include <cstdio>
#include <vector>

using namespace GSView::oracle;

namespace {

OracleCamera frontalCamera()
{
    OracleCamera c;
    c.viewRot = glm::dmat3(1.0);           // world == camera
    c.viewPos = glm::dvec3(0.0, 0.0, 0.0); // at the origin looking +Z
    c.focalX = c.focalY = 200.0;
    return c;
}

Gaussian3D isoSplat(double z, double opacity, double sigmaWorld = 0.15)
{
    Gaussian3D g;
    g.pos = glm::dvec3(0.0, 0.0, z);
    g.logScale = glm::dvec3(std::log(sigmaWorld));
    g.opacity = opacity;
    g.color = glm::dvec3(0.9, 0.4, 0.2);
    return g;
}

constexpr int    kW = 96, kH = 96;
constexpr int    kSppSide = 2;
constexpr int    kSets = 300;
const glm::dvec3 kBg(0.05, 0.05, 0.08);

} // namespace

TEST(GaussianPointOracle, SingleGaussianConvergesToAnalytic)
{
    for (double o : { 0.1, 0.5, 0.9 }) {
        const std::vector<Gaussian3D> scene = { isoSplat(6.0, o) };
        const auto ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
        const auto mc  = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg);

        const double p = psnr(mc, ref);
        const double s = ssim(mc, ref, kW, kH);
        std::printf("[oracle] single Gaussian o=%.1f  PSNR=%.2f dB  SSIM=%.4f\n", o, p, s);
        EXPECT_GE(p, 35.0) << "opacity " << o << " PSNR " << p;
        EXPECT_GE(s, 0.98) << "opacity " << o << " SSIM " << s;
    }
}

TEST(GaussianPointOracle, FrontBackTwoLayerConvergesToAnalytic)
{
    std::vector<Gaussian3D> scene;
    Gaussian3D back = isoSplat(9.0, 0.8);
    back.color = glm::dvec3(0.15, 0.7, 0.25);
    Gaussian3D front = isoSplat(5.0, 0.6);
    front.color = glm::dvec3(0.9, 0.2, 0.2);
    scene = { back, front };

    const auto ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
    const auto mc  = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg);

    const double p = psnr(mc, ref);
    const double s = ssim(mc, ref, kW, kH);
    std::printf("[oracle] front/back two-layer  PSNR=%.2f dB  SSIM=%.4f\n", p, s);
    EXPECT_GE(p, 35.0) << "PSNR " << p;
    EXPECT_GE(s, 0.98) << "SSIM " << s;

    // Order independence: reversing the input must give a statistically equal image.
    std::vector<Gaussian3D> reversed = { front, back };
    const auto mc2 = renderMonteCarlo(reversed, frontalCamera(), kW, kH, kSppSide, kSets, kBg);
    EXPECT_GE(psnr(mc, mc2), 40.0);
}

TEST(GaussianPointOracle, AnisotropicRotatedGaussianConvergesToAnalytic)
{
    Gaussian3D g;
    g.pos = glm::dvec3(0.0, 0.0, 6.0);
    g.logScale = glm::dvec3(std::log(0.28), std::log(0.10), std::log(0.06));
    g.rot = glm::normalize(glm::dquat(0.82, 0.20, -0.45, 0.30));
    g.opacity = 0.7;
    g.color = glm::dvec3(0.3, 0.5, 0.95);
    const std::vector<Gaussian3D> scene = { g };

    const auto ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
    const auto mc  = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg);

    const double p = psnr(mc, ref), s = ssim(mc, ref, kW, kH);
    std::printf("[oracle] anisotropic rotated   PSNR=%.2f dB  SSIM=%.4f\n", p, s);
    EXPECT_GE(p, 35.0);
    EXPECT_GE(s, 0.98);
}

TEST(GaussianPointOracle, FullyTransparentSceneIsBackground)
{
    const std::vector<Gaussian3D> scene = { isoSplat(6.0, 0.0) };
    const auto mc = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, 8, kBg);
    for (const auto& px : mc) {
        EXPECT_NEAR(px.r, kBg.r, 1e-9);
        EXPECT_NEAR(px.g, kBg.g, 1e-9);
        EXPECT_NEAR(px.b, kBg.b, 1e-9);
    }
}

TEST(GaussianPointOracle, MetricsSanity)
{
    Image a(64 * 64, glm::dvec3(0.5));
    Image b = a;
    EXPECT_GE(psnr(a, b), 100.0);
    EXPECT_NEAR(ssim(a, b, 64, 64), 1.0, 1e-9);

    for (auto& px : b) px += glm::dvec3(0.1);
    EXPECT_LT(psnr(a, b), 25.0);
}
