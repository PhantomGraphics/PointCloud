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

// CPU port of the GPU pipeline. `numSets` independent sample sets are averaged
// (each with a distinct frame seed), mirroring progressive accumulation.
Image renderMonteCarlo(const std::vector<Gaussian3D>& scene,
                       OracleCamera cam, int W, int H, int sppSide, int numSets,
                       const glm::dvec3& background, std::uint32_t baseSeed = 1u);

// Metrics over linear-RGB images (values expected in [0,1]).
double psnr(const Image& a, const Image& b);
double ssim(const Image& a, const Image& b, int W, int H);

} // namespace GSView::oracle
