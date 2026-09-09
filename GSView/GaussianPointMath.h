#pragma once

// -----------------------------------------------------------------------------
// GaussianPointMath -- pure CPU maths for the Gaussian-Point / PBVR3D research
// paths (docs/todo/PLAN_gsview_gaussian_point_pbvr.md, Phase 1).
//
// Everything here is a deterministic pure function with no Vulkan / renderer
// dependency, so it can back both the CPU oracle (PointCloudTest) and, later,
// the GPU shaders (as the reference the shader maths is checked against).
//
// Computations are done in double for oracle precision; GPU code will use float
// and is expected to agree only to a loose tolerance.
// -----------------------------------------------------------------------------

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace GSView::gpm {

// SH band-0 basis value: color = f_dc * kShC0 + 0.5. Centralised so the CPU
// reference, the GPU shaders and the loaders all agree on one constant.
inline constexpr double kShC0 = 0.28209479177387814;

// pi^2 / 6 = Li2(1), the supremum of the dilogarithm on [0,1].
inline constexpr double kDilogAtOne = 1.6449340668482264;

// ---------------------------------------------------------------------------
// Scalar activations
// ---------------------------------------------------------------------------

double sigmoid(double x);

// clamp(sh * kShC0 + 0.5, 0, 1)
double shDcToColor(double sh);

// ---------------------------------------------------------------------------
// Covariance
// ---------------------------------------------------------------------------

// 3D covariance Sigma = R S^2 R^T, with S = diag(exp(logScale)) and R the
// rotation of the (w,x,y,z) quaternion (normalised internally).
glm::dmat3 covariance3D(const glm::dvec3& logScale, const glm::dquat& rot);

// Rotation matrix of a (w,x,y,z) quaternion, normalised internally.
glm::dmat3 quatToRotation(const glm::dquat& rot);

// Pinhole camera used for the screen-space covariance projection.
//   worldToCam(p) = viewRot * (p - viewPos)   -- camera looks along +Z,
//   so a visible point has cameraSpace.z > 0.
struct PinholeCamera {
    glm::dmat3 viewRot{ 1.0 };   // world -> camera rotation
    glm::dvec3 viewPos{ 0.0 };   // camera position, world space
    double     focalX = 1.0;     // pixels
    double     focalY = 1.0;     // pixels
    double     tanFovX = 1.0;    // half-fov tangents, for the frustum clamp
    double     tanFovY = 1.0;
};

glm::dvec3 worldToCamera(const PinholeCamera& cam, const glm::dvec3& world);

// EWA screen-space covariance: cov2d = block2x2( (J W) Sigma3d (J W)^T ), where
// J is the perspective Jacobian at the (frustum-clamped) camera-space mean and
// W = cam.viewRot. `lowPass` is the 3DGS antialiasing dilation added to the
// diagonal (0.3 px^2 in the reference; pass 0 to disable).
glm::dmat2 covariance2D(const glm::dmat3& cov3d,
                        const glm::dvec3& meanWorld,
                        const PinholeCamera& cam,
                        double lowPass = 0.3);

// Lower-triangular L with L L^T = A (2x2, SPD). Used to map whitened samples
// back into pixel space.
glm::dmat2 cholesky2D(const glm::dmat2& A);

// ---------------------------------------------------------------------------
// Dilogarithm  Li2(x) = sum_{k>=1} x^k / k^2  ( -1 <= x <= 1 )
// ---------------------------------------------------------------------------

double dilog(double x);

// Inverse of dilog restricted to [0,1) -> [0, pi^2/6): returns y with
// dilog(y) == target. `target` is clamped to [0, kDilogAtOne).
double invDilog(double target);

// ---------------------------------------------------------------------------
// Expected point count
//   lambda(x) = -log(1 - alpha(x)),  alpha(x) = o * exp(-1/2 d^T Sigma2d^-1 d)
//   E[N] = integral lambda dx = 2*pi*sqrt(det Sigma2d) * Li2(o)
// (per unit pixel area and per pass; the caller multiplies by spp / area).
// ---------------------------------------------------------------------------

double expectedPointCount(double detCov2d, double opacity);

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

// pcg hash, bit-identical to gs_pbvr_gen.comp's pcg() so CPU and GPU can share
// seed streams.
std::uint32_t pcgHash(std::uint32_t v);

enum class SeedMode {
    Deterministic,   // depends only on (splatId, particleIndex)
    FrameVarying     // also folds in frameIndex, for progressive accumulation
};

// One 32-bit seed for a given particle. In Deterministic mode `frameIndex` is
// ignored; in FrameVarying mode two different frames give independent streams.
std::uint32_t particleSeed(SeedMode mode,
                           std::uint32_t splatId,
                           std::uint32_t particleIndex,
                           std::uint32_t frameIndex);

// A tiny stateful uniform stream seeded by particleSeed(); values in [0,1).
class RandStream {
public:
    explicit RandStream(std::uint32_t seed) : state_(seed ? seed : 0x9E3779B9u) {}
    double next();                    // U[0,1)
    double normal();                  // N(0,1), Box-Muller (one value per call)
private:
    std::uint32_t state_;
    bool   haveSpare_ = false;
    double spare_ = 0.0;
};

// Poisson(lambda) via Knuth for small lambda, normal approximation for large.
int poissonSample(double lambda, RandStream& rng);

// Whitened radius r for the corrected radial distribution whose density is
// proportional to  r * (-log(1 - o * exp(-r^2/2))). `u` is U[0,1).
// Derivation: F(r) = 1 - Li2(o e^{-r^2/2}) / Li2(o).
double sampleCorrectedRadius(double opacity, double u);

// A 2D offset (pixels) around the mean: draws r from sampleCorrectedRadius and a
// uniform angle, then maps through the Cholesky factor of cov2d.
glm::dvec2 sampleCorrectedOffset(double opacity, const glm::dmat2& cov2d, RandStream& rng);

// ---------------------------------------------------------------------------
// Analytic per-pixel coverage oracle
// ---------------------------------------------------------------------------

// A projected Gaussian in pixel space.
struct Gaussian2D {
    glm::dvec2 mean{ 0.0 };
    glm::dmat2 cov{ 1.0 };
    double     opacity = 1.0;   // o in [0,1]
    glm::dvec3 color{ 1.0 };    // linear RGB
    double     depth = 0.0;     // for compositing order (smaller = nearer)
};

// alpha(pixel) = o * exp(-1/2 d^T cov^-1 d)
double alphaAt(const Gaussian2D& g, const glm::dvec2& pixel);

// Expected coverage of a stochastic-opaque-point pass over a pixel of the given
// area: 1 - (1 - alpha)^pixelArea. For pixelArea == 1 this is exactly alpha --
// that identity is the whole reason the point density is -log(1-alpha).
double expectedCoverage(double alpha, double pixelArea = 1.0);

enum class AlphaMode {
    Straight,        // rgb is un-multiplied; blend = SRC_ALPHA / ONE_MINUS_SRC_ALPHA
    Premultiplied    // rgb already multiplied by alpha; blend = ONE / ONE_MINUS_SRC_ALPHA
};

// "src over dst". Both operands and the result follow `mode`.
glm::dvec4 over(const glm::dvec4& src, const glm::dvec4& dst, AlphaMode mode);

// Expected composited colour of a stack of projected Gaussians at one pixel,
// front-to-back, using expectedCoverage() as each layer's alpha over `background`.
// `layers` need not be sorted; this sorts by depth ascending (near first).
glm::dvec3 compositeExpected(std::vector<Gaussian2D> layers,
                             const glm::dvec2& pixel,
                             const glm::dvec3& background,
                             double pixelArea = 1.0);

// ---------------------------------------------------------------------------
// Depth / colour packing for atomicMin sample buffers
// ---------------------------------------------------------------------------

// Maps an IEEE-754 float to a uint32 that preserves ordering for non-negative
// values (so a plain unsigned atomicMin keeps the nearest sample).
std::uint32_t orderedDepthKey(float depth);
float         orderedDepthKeyInverse(std::uint32_t key);

// (depthKey << 32) | rgba8 -- the 64-bit key an atomicMin race resolves.
std::uint64_t packDepthColor(float depth, std::uint32_t rgba8);
std::uint32_t unpackColor(std::uint64_t packed);
std::uint32_t unpackDepthKey(std::uint64_t packed);

} // namespace GSView::gpm
