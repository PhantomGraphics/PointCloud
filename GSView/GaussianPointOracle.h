#pragma once

// -----------------------------------------------------------------------------
// GaussianPointOracle -- CPU rendering + image metrics for verification
// (docs/todo/PLAN_gsview_gaussian_point_pbvr.md Phase 6).
//
// Two renderers over the same analytic scene:
//   renderAnalytic()   -- the ground truth: per-pixel expected coverage-composited
//                          colour (GaussianPointMath::compositeExpected).
//   renderMonteCarlo()  -- a CPU port of the GPU pipeline (gps_splat.comp +
//                          gps_resolve.comp): project each Gaussian, Poisson-count
//                          points, scatter corrected offsets into subpixel cells,
//                          keep the nearest, resolve, average over sample sets.
//
// If renderMonteCarlo converges to renderAnalytic (PSNR / SSIM) then the
// algorithm the GPU implements is unbiased and matches the oracle. Camera looks
// +Z (cam.z > 0 for visible points); the convention is internal to this file, so
// only self-consistency matters.
// -----------------------------------------------------------------------------

#include "GaussianPointMath.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <vector>

namespace GSView::oracle {

struct Gaussian3D {
    glm::dvec3 pos{ 0.0 };
    glm::dvec3 logScale{ 0.0 };          // log of the per-axis std-dev
    glm::dquat rot{ 1.0, 0.0, 0.0, 0.0 }; // (w,x,y,z)
    double     opacity = 1.0;            // activated, in [0,1]
    glm::dvec3 color{ 1.0 };             // linear RGB
};

struct OracleCamera {
    glm::dmat3 viewRot{ 1.0 };
    glm::dvec3 viewPos{ 0.0 };
    double     focalX = 200.0, focalY = 200.0;
    double     cx = 0.0, cy = 0.0;       // principal point (pixels); default set by render*()
    double     lowPass = 0.3;
};

using Image = std::vector<glm::dvec3>;   // row-major, size W*H

// Ground truth.
Image renderAnalytic(const std::vector<Gaussian3D>& scene,
                     OracleCamera cam, int W, int H,
                     const glm::dvec3& background);

Image renderAnalyticSamples(const std::vector<Gaussian3D>& scene,
                            OracleCamera cam, int W, int H,
                            const glm::dvec3& background,
                            const std::vector<glm::ivec2>& pixels);

// CPU port of the GPU pipeline. `numSets` independent sample sets are averaged
// (each with a distinct frame seed), mirroring progressive accumulation.
Image renderMonteCarlo(const std::vector<Gaussian3D>& scene,
                       OracleCamera cam, int W, int H, int sppSide, int numSets,
                       const glm::dvec3& background, std::uint32_t baseSeed = 1u);

// ---------------------------------------------------------------------------
// Footprint-aware density calibration (docs/todo/PLAN_footprint_aware_density_calibration.md
// Phase 2; theory in docs/paper/NOTE_footprint_density_calibration.md).
// ---------------------------------------------------------------------------

// Totals over all sample sets of one render, for cost comparisons.
struct RenderStats {
    double points = 0.0;         // generated points / accepted particles
    double candidates = 0.0;     // particles before radial thinning (renderParticles3D)
    double subpixelWrites = 0.0; // depth tests performed (one per covered subpixel)
};

// Variable-footprint points (level F) for the GPS Monte Carlo port.
struct MonteCarloOptions {
    int    footprintSubpixels = 1;   // fixed s: each point covers an s x s subpixel block
    double adaptiveKappa = 0.0;      // > 0: per-splat s_i = gpm::adaptiveFootprint(sigmaMin, kappa, footprintMax)
    int    footprintMax = 4;
    // Subtract the block's variance (s/side)^2/12 from the projected covariance so the
    // blurred opacity keeps its second moment. Skipped for splats it would make degenerate.
    bool   compensateBlur = false;
    RenderStats* stats = nullptr;    // optional totals
};

// renderMonteCarlo() with point footprints. With the default options this is
// bit-identical to the overload above (same RNG stream, same writes).
Image renderMonteCarlo(const std::vector<Gaussian3D>& scene,
                       OracleCamera cam, int W, int H, int sppSide, int numSets,
                       const glm::dvec3& background, std::uint32_t baseSeed,
                       const MonteCarloOptions& options);

// CPU model of the PBVR3D path: world-space particles drawn from each 3D Gaussian,
// counted by a calibration level, projected, and resolved exactly like
// renderMonteCarlo(). The ablation switches isolate the two residuals that remain
// after C3+R (NOTE Sec. 3.3).
struct ParticleOptions {
    gpm::CalibrationLevel level = gpm::CalibrationLevel::PerSplatFootprint;
    gpm::OpacityRule      rule  = gpm::OpacityRule::Extinction;
    bool   radialCorrection = false;     // C3+R: keep with gpm::radialKeepProbability
    double baseK = 512.0;                // C0-C2 base count
    double referencePixelLength = 0.01;  // C1/C2 l0
    double nearZ = 0.05;
    // Add N(0, lowPass I) to each projected position so the particle cloud has the
    // same 2D covariance as the GPS footprint (which includes the low-pass term).
    bool   lowPassJitter = true;
    // Ablations: give every particle its splat-centre depth (the GPS depth rule) and/or
    // project through the EWA linearisation instead of exact perspective.
    bool   centreDepth = false;
    bool   linearizedProjection = false;
    RenderStats* stats = nullptr;
};

Image renderParticles3D(const std::vector<Gaussian3D>& scene,
                        OracleCamera cam, int W, int H, int sppSide, int numSets,
                        const glm::dvec3& background, std::uint32_t baseSeed,
                        const ParticleOptions& options);

// Metrics over linear-RGB images (values expected in [0,1]).
double psnr(const Image& a, const Image& b);
double ssim(const Image& a, const Image& b, int W, int H);

} // namespace GSView::oracle
