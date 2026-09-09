#pragma once

// -----------------------------------------------------------------------------
// Shared CPU/GPU test vectors for the Gaussian-Point maths
// (docs/todo/PLAN_gsview_gaussian_point_pbvr.md, Phase 1, completion criterion
// "CPU/GPU 共通テストベクトルを JSON または固定配列で共有できる").
//
// Fixed C++ array rather than JSON: no parser dependency, and the same header
// can be #included by PointCloudTest now and by the GPU-side check later.
//
// Each case gives the inputs plus values derived analytically / by hand (shown
// in the comment), NOT by calling GaussianPointMath -- so a test comparing the
// implementation against this table is a real check, not a tautology.
// -----------------------------------------------------------------------------

#include <cstdint>

namespace GSView::gpm::vectors {

// --- Dilogarithm ----------------------------------------------------------
// Li2(0)   = 0
// Li2(1/2) = pi^2/12 - (ln 2)^2 / 2      = 0.5822405264650125
// Li2(1)   = pi^2/6                      = 1.6449340668482264
// Li2(-1)  = -pi^2/12                    = -0.8224670334241132
struct DilogCase { double x; double li2; };
inline constexpr DilogCase kDilog[] = {
    { 0.0,  0.0 },
    { 0.5,  0.5822405264650125 },
    { 1.0,  1.6449340668482264 },
    { -1.0, -0.8224670334241132 },
    { 0.25, 0.2676526390827325 },   // sum_k 0.25^k/k^2
    { 0.9,  1.2997147230049588 },
};

// --- sigmoid ------------------------------------------------------------
struct SigmoidCase { double x; double s; };
inline constexpr SigmoidCase kSigmoid[] = {
    {  0.0, 0.5 },
    {  1.0, 0.7310585786300049 },
    { -2.0, 0.11920292202211755 },
    { 40.0, 1.0 },                   // saturates
};

// --- Expected point count ---------------------------------------------------
// E[N] = 2*pi*sqrt(det Sigma2d) * Li2(o).
// Isotropic screen-space covariance det = s2^2 (Sigma2d = s2 * I).
struct ExpectedCountCase {
    double cov2dIsotropic;   // the single value s2 on Sigma2d's diagonal
    double opacity;          // o
    double expectedN;        // 2*pi*s2*Li2(o)
};
inline constexpr ExpectedCountCase kExpectedCount[] = {
    // s2 = 1, o = 1   -> 2*pi*1*1.6449340668482264 = 10.335425560099939
    { 1.0, 1.0, 10.335425560099939 },
    // s2 = 4, o = 0.5 -> 2*pi*4*0.5822405264650125 = 14.633300484517893
    { 4.0, 0.5, 14.633300484517893 },
    // s2 = 2.5, o = 0.9 -> 2*pi*2.5*1.2997147230049588 = 20.41587112777436
    { 2.5, 0.9, 20.41587112777436 },
};

// --- Ordered depth key ----------------------------------------------------
// orderedDepthKey maps a float to a uint32 monotone in the value (for >= 0).
// For positive floats: key = floatBits | 0x80000000.
struct DepthKeyCase { float depth; std::uint32_t key; };
inline constexpr DepthKeyCase kDepthKey[] = {
    { 0.0f,   0x80000000u },
    { 1.0f,   0xBF800000u },   // 0x3F800000 | 0x80000000
    { 2.0f,   0xC0000000u },   // 0x40000000 | 0x80000000
    { 100.0f, 0xC2C80000u },   // 0x42C80000 | 0x80000000
};

} // namespace GSView::gpm::vectors
