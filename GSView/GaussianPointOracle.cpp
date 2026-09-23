#include "GaussianPointOracle.h"

#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <cmath>

namespace GSView::oracle {

namespace gpm = GSView::gpm;

namespace {

gpm::PinholeCamera toPinhole(const OracleCamera& c)
{
    gpm::PinholeCamera p;
    p.viewRot = c.viewRot;
    p.viewPos = c.viewPos;
    p.focalX = c.focalX;
    p.focalY = c.focalY;
    // tan(fov/2) = (extent/2) / focal; only used for the EWA frustum clamp, and a
    // generous value keeps well-centred test scenes unclamped.
    p.tanFovX = 4.0;
    p.tanFovY = 4.0;
    return p;
}

struct Projected {
    bool       visible = false;
    glm::dvec2 mean{ 0.0 };
    glm::dmat2 cov2d{ 1.0 };
    double     detCov2d = 0.0;
    double     depth = 0.0;      // camera-space z (smaller = nearer)
    double     opacity = 0.0;
    glm::dvec3 color{ 0.0 };
};

Projected project(const Gaussian3D& g, const OracleCamera& c)
{
    Projected pr;
    const gpm::PinholeCamera pin = toPinhole(c);
    const glm::dvec3 camMean = gpm::worldToCamera(pin, g.pos);
    if (camMean.z <= 1e-4) return pr;

    const glm::dmat3 cov3d = gpm::covariance3D(g.logScale, g.rot);
    pr.cov2d = gpm::covariance2D(cov3d, g.pos, pin, c.lowPass);
    pr.detCov2d = glm::determinant(pr.cov2d);
    if (pr.detCov2d <= 0.0) return pr;

    pr.mean = glm::dvec2(c.focalX * camMean.x / camMean.z + c.cx,
                         c.focalY * camMean.y / camMean.z + c.cy);
    pr.depth = camMean.z;
    pr.opacity = std::clamp(g.opacity, 0.0, 1.0);
    pr.color = g.color;
    pr.visible = true;
    return pr;
}

void defaultPrincipalPoint(OracleCamera& c, int W, int H)
{
    if (c.cx == 0.0 && c.cy == 0.0) {
        c.cx = 0.5 * W;
        c.cy = 0.5 * H;
    }
}

std::vector<gpm::Gaussian2D> projectLayers(const std::vector<Gaussian3D>& scene,
                                           const OracleCamera& cam)
{
    std::vector<gpm::Gaussian2D> layers;
    layers.reserve(scene.size());
    for (const auto& g : scene) {
        const Projected p = project(g, cam);
        if (!p.visible) continue;
        gpm::Gaussian2D layer;
        layer.mean = p.mean;
        layer.cov = p.cov2d;
        layer.opacity = p.opacity;
        layer.color = p.color;
        layer.depth = p.depth;
        layers.push_back(layer);
    }
    return layers;
}

// Nearest-wins depth test on one subpixel (the CPU stand-in for the GPU atomicMin).
void depthTest(size_t idx, double d, const glm::dvec3& c,
               std::vector<double>& depth, std::vector<glm::dvec3>& color, RenderStats* stats)
{
    if (stats) stats->subpixelWrites += 1.0;
    if (d < depth[idx]) {
        depth[idx] = d;
        color[idx] = c;
    }
}

// An s x s block of subpixels centred on `pixel` (pixel units): the cells whose
// centres lie in [c - s/2, c + s/2) along each axis, c = pixel * sppSide. For s = 1
// this is exactly the subpixel containing `pixel`.
void writeBlock(const glm::dvec2& pixel, int s, int W, int H, int sppSide, double d,
                const glm::dvec3& c, std::vector<double>& depth, std::vector<glm::dvec3>& color,
                RenderStats* stats)
{
    const int spp = sppSide * sppSide;
    const long long gx0 = static_cast<long long>(std::floor(pixel.x * sppSide - 0.5 * s + 0.5));
    const long long gy0 = static_cast<long long>(std::floor(pixel.y * sppSide - 0.5 * s + 0.5));
    const long long gw = static_cast<long long>(W) * sppSide;
    const long long gh = static_cast<long long>(H) * sppSide;
    for (long long gy = std::max(0LL, gy0); gy < std::min(gh, gy0 + s); ++gy) {
        for (long long gx = std::max(0LL, gx0); gx < std::min(gw, gx0 + s); ++gx) {
            const long long px = gx / sppSide, py = gy / sppSide;
            const long long sx = gx % sppSide, sy = gy % sppSide;
            const size_t idx = (static_cast<size_t>(py) * W + static_cast<size_t>(px)) * spp
                             + static_cast<size_t>(sy * sppSide + sx);
            depthTest(idx, d, c, depth, color, stats);
        }
    }
}

void resolveSet(int W, int H, int spp, const std::vector<double>& depth,
                const std::vector<glm::dvec3>& color, const glm::dvec3& background, Image& accum)
{
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const size_t base = (static_cast<size_t>(y) * W + x) * spp;
            glm::dvec3 sum(0.0);
            for (int k = 0; k < spp; ++k)
                sum += (depth[base + k] < 1e299) ? color[base + k] : background;
            accum[static_cast<size_t>(y) * W + x] += sum / static_cast<double>(spp);
        }
    }
}

} // namespace

Image renderAnalytic(const std::vector<Gaussian3D>& scene, OracleCamera cam,
                     int W, int H, const glm::dvec3& background)
{
    defaultPrincipalPoint(cam, W, H);

    const std::vector<gpm::Gaussian2D> layers = projectLayers(scene, cam);

    Image img(static_cast<size_t>(W) * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const glm::dvec2 p(x + 0.5, y + 0.5);
            img[static_cast<size_t>(y) * W + x] =
                gpm::compositeExpected(layers, p, background, 1.0);
        }
    }
    return img;
}

Image renderAnalyticSamples(const std::vector<Gaussian3D>& scene, OracleCamera cam,
                            int W, int H, const glm::dvec3& background,
                            const std::vector<glm::ivec2>& pixels)
{
    defaultPrincipalPoint(cam, W, H);
    const std::vector<gpm::Gaussian2D> layers = projectLayers(scene, cam);
    Image img;
    img.reserve(pixels.size());
    for (const glm::ivec2& px : pixels) {
        if (px.x < 0 || px.x >= W || px.y < 0 || px.y >= H) {
            img.push_back(background);
            continue;
        }
        img.push_back(gpm::compositeExpected(
            layers, glm::dvec2(px.x + 0.5, px.y + 0.5), background, 1.0));
    }
    return img;
}

Image renderMonteCarlo(const std::vector<Gaussian3D>& scene, OracleCamera cam,
                       int W, int H, int sppSide, int numSets,
                       const glm::dvec3& background, std::uint32_t baseSeed)
{
    return renderMonteCarlo(scene, cam, W, H, sppSide, numSets, background, baseSeed,
                            MonteCarloOptions{});
}

Image renderMonteCarlo(const std::vector<Gaussian3D>& scene, OracleCamera cam,
                       int W, int H, int sppSide, int numSets,
                       const glm::dvec3& background, std::uint32_t baseSeed,
                       const MonteCarloOptions& options)
{
    defaultPrincipalPoint(cam, W, H);
    sppSide = std::clamp(sppSide, 1, 4);
    const int spp = sppSide * sppSide;
    const size_t nSub = static_cast<size_t>(W) * H * spp;

    std::vector<Projected> proj;
    proj.reserve(scene.size());
    for (const auto& g : scene) {
        Projected p = project(g, cam);
        if (p.visible) proj.push_back(p);
    }

    // Per-splat footprint and the covariance points are sampled from (the projected
    // covariance, optionally shrunk by the block variance).
    std::vector<int>        footprint(proj.size(), 1);
    std::vector<glm::dmat2> sampleCov(proj.size());
    for (size_t g = 0; g < proj.size(); ++g) {
        const Projected& pr = proj[g];
        int fp = std::max(1, options.footprintSubpixels);
        if (options.adaptiveKappa > 0.0) {
            const double tr = pr.cov2d[0][0] + pr.cov2d[1][1];
            const double disc = std::sqrt(std::max(0.0, 0.25 * tr * tr - pr.detCov2d));
            const double sigmaMinPx = std::sqrt(std::max(0.0, 0.5 * tr - disc));
            fp = gpm::adaptiveFootprint(sigmaMinPx * sppSide, options.adaptiveKappa, options.footprintMax);
        }
        footprint[g] = fp;
        sampleCov[g] = pr.cov2d;
        if (options.compensateBlur && fp > 1) {
            const double v = (static_cast<double>(fp) / sppSide) * (fp / static_cast<double>(sppSide)) / 12.0;
            const glm::dmat2 shrunk = pr.cov2d - glm::dmat2(v, 0.0, 0.0, v);
            const double t = shrunk[0][0] + shrunk[1][1];
            const double d = glm::determinant(shrunk);
            // Keep it only while it stays comfortably SPD (both eigenvalues >= v).
            if (d > 0.0 && t > 0.0 && 0.5 * t - std::sqrt(std::max(0.0, 0.25 * t * t - d)) >= v)
                sampleCov[g] = shrunk;
        }
    }

    Image accum(static_cast<size_t>(W) * H, glm::dvec3(0.0));

    std::vector<double>     depth(nSub);
    std::vector<glm::dvec3> color(nSub);

    for (int s = 0; s < numSets; ++s) {
        std::fill(depth.begin(), depth.end(), 1e300);
        std::fill(color.begin(), color.end(), glm::dvec3(0.0));

        for (std::uint32_t gid = 0; gid < proj.size(); ++gid) {
            const Projected& pr = proj[gid];
            const int fp = footprint[gid];

            // The count keeps the unshrunk footprint's total optical mass; only the
            // positions below come from the (optionally blur-compensated) covariance.
            const double EN = gpm::footprintPointCount(
                gpm::expectedPointCount(pr.detCov2d, pr.opacity) * static_cast<double>(spp), fp);
            if (EN <= 0.0) continue;

            const std::uint32_t seed = gpm::particleSeed(
                gpm::SeedMode::FrameVarying, gid, 0u,
                baseSeed + static_cast<std::uint32_t>(s));
            gpm::RandStream rng(seed);
            const int N = gpm::poissonSample(EN, rng);
            if (options.stats) options.stats->points += N;

            for (int i = 0; i < N; ++i) {
                const glm::dvec2 off = gpm::sampleCorrectedOffset(pr.opacity, sampleCov[gid], rng);
                const glm::dvec2 pixel(pr.mean.x + off.x, pr.mean.y + off.y);
                if (fp == 1) {
                    if (pixel.x < 0.0 || pixel.x >= W || pixel.y < 0.0 || pixel.y >= H) continue;
                    const int px = static_cast<int>(pixel.x);
                    const int py = static_cast<int>(pixel.y);
                    const int sx = std::min(sppSide - 1, static_cast<int>((pixel.x - px) * sppSide));
                    const int sy = std::min(sppSide - 1, static_cast<int>((pixel.y - py) * sppSide));
                    depthTest((static_cast<size_t>(py) * W + px) * spp + (sy * sppSide + sx),
                              pr.depth, pr.color, depth, color, options.stats);
                } else {
                    writeBlock(pixel, fp, W, H, sppSide, pr.depth, pr.color, depth, color, options.stats);
                }
            }
        }

        resolveSet(W, H, spp, depth, color, background, accum);
    }

    for (auto& c : accum) c /= static_cast<double>(numSets);
    return accum;
}

Image renderParticles3D(const std::vector<Gaussian3D>& scene, OracleCamera cam,
                        int W, int H, int sppSide, int numSets,
                        const glm::dvec3& background, std::uint32_t baseSeed,
                        const ParticleOptions& options)
{
    defaultPrincipalPoint(cam, W, H);
    sppSide = std::clamp(sppSide, 1, 4);
    const int spp = sppSide * sppSide;
    const size_t nSub = static_cast<size_t>(W) * H * spp;
    const gpm::PinholeCamera pin = toPinhole(cam);

    // Object centre for C1: the arithmetic mean of the Gaussian centres, as the renderer.
    glm::dvec3 centre(0.0);
    for (const auto& g : scene) centre += g.pos;
    if (!scene.empty()) centre /= static_cast<double>(scene.size());
    const double objectDepth = gpm::worldToCamera(pin, centre).z;

    struct Prep {
        const Gaussian3D* g = nullptr;
        Projected pr;
        glm::dmat3 L3{ 1.0 };      // Cholesky factor of the 3D covariance
        glm::dmat2 cov2dInv{ 1.0 };
        glm::dmat3 JW{ 0.0 };      // rows 0/1: linearised projection d(pixel)/d(world)
        double lambda = 0.0;       // expected candidate count per set
    };
    std::vector<Prep> preps;
    preps.reserve(scene.size());
    for (const auto& g : scene) {
        Prep p;
        p.g = &g;
        p.pr = project(g, cam);
        if (!p.pr.visible) continue;
        const glm::dmat3 cov3 = gpm::covariance3D(g.logScale, g.rot);
        // 3x3 Cholesky (column-major glm: m[col][row]).
        const double a00 = cov3[0][0], a10 = cov3[0][1], a20 = cov3[0][2];
        const double a11 = cov3[1][1], a21 = cov3[1][2], a22 = cov3[2][2];
        const double l00 = std::sqrt(std::max(a00, 1e-30));
        const double l10 = a10 / l00, l20 = a20 / l00;
        const double l11 = std::sqrt(std::max(a11 - l10 * l10, 1e-30));
        const double l21 = (a21 - l20 * l10) / l11;
        const double l22 = std::sqrt(std::max(a22 - l20 * l20 - l21 * l21, 1e-30));
        p.L3 = glm::dmat3(glm::dvec3(l00, l10, l20), glm::dvec3(0.0, l11, l21), glm::dvec3(0.0, 0.0, l22));
        p.cov2dInv = glm::inverse(p.pr.cov2d);

        const glm::dvec3 cm = gpm::worldToCamera(pin, g.pos);
        const double iz = 1.0 / cm.z;
        // J (2x3, pixel per camera unit) times W = viewRot, stored as the first two rows.
        const glm::dvec3 jx(cam.focalX * iz, 0.0, -cam.focalX * cm.x * iz * iz);
        const glm::dvec3 jy(0.0, cam.focalY * iz, -cam.focalY * cm.y * iz * iz);
        const glm::dmat3 Wt = glm::transpose(cam.viewRot);   // columns = rows of viewRot
        p.JW = glm::dmat3(0.0);
        const glm::dvec3 rx = Wt * jx, ry = Wt * jy;         // row vectors of J*W
        p.JW[0] = glm::dvec3(rx.x, ry.x, 0.0);
        p.JW[1] = glm::dvec3(rx.y, ry.y, 0.0);
        p.JW[2] = glm::dvec3(rx.z, ry.z, 0.0);

        gpm::CalibrationInputs in;
        in.objectDepth = objectDepth;
        in.splatDepth = cm.z;
        in.focalX = cam.focalX;
        in.focalY = cam.focalY;
        in.nearZ = options.nearZ;
        in.referencePixelLength = options.referencePixelLength;
        in.footprintArea = gpm::projectedFootprintArea(p.pr.cov2d);
        in.spp = static_cast<double>(spp);
        p.lambda = gpm::calibratedCount(options.level, options.rule, p.pr.opacity,
                                        options.baseK, 1.0, in);
        preps.push_back(p);
    }

    Image accum(static_cast<size_t>(W) * H, glm::dvec3(0.0));
    std::vector<double>     depth(nSub);
    std::vector<glm::dvec3> color(nSub);
    const glm::dvec3 forward = glm::transpose(cam.viewRot)[2];   // camera +Z in world space
    const double jitter = options.lowPassJitter ? std::sqrt(std::max(0.0, cam.lowPass)) : 0.0;

    for (int s = 0; s < numSets; ++s) {
        std::fill(depth.begin(), depth.end(), 1e300);
        std::fill(color.begin(), color.end(), glm::dvec3(0.0));

        for (std::uint32_t gid = 0; gid < preps.size(); ++gid) {
            const Prep& p = preps[gid];
            if (p.lambda <= 0.0) continue;
            gpm::RandStream rng(gpm::particleSeed(gpm::SeedMode::FrameVarying, gid, 1u,
                                                  baseSeed + static_cast<std::uint32_t>(s)));
            const int N = gpm::poissonSample(p.lambda, rng);
            if (options.stats) options.stats->candidates += N;

            for (int i = 0; i < N; ++i) {
                const glm::dvec3 u(rng.normal(), rng.normal(), rng.normal());
                const glm::dvec3 world = p.g->pos + p.L3 * u;
                glm::dvec2 pixel;
                double d;
                if (options.linearizedProjection) {
                    const glm::dvec3 off = p.JW * (world - p.g->pos);
                    pixel = p.pr.mean + glm::dvec2(off.x, off.y);
                    d = p.pr.depth + glm::dot(forward, world - p.g->pos);
                } else {
                    const glm::dvec3 cp = gpm::worldToCamera(pin, world);
                    if (cp.z <= options.nearZ) continue;
                    pixel = glm::dvec2(cam.focalX * cp.x / cp.z + cam.cx, cam.focalY * cp.y / cp.z + cam.cy);
                    d = cp.z;
                }
                if (jitter > 0.0) pixel += jitter * glm::dvec2(rng.normal(), rng.normal());
                if (options.radialCorrection) {
                    const glm::dvec2 dv = pixel - p.pr.mean;
                    const double r2 = glm::dot(dv, p.cov2dInv * dv);
                    if (rng.next() >= gpm::radialKeepProbability(p.pr.opacity, r2)) continue;
                }
                if (options.centreDepth) d = p.pr.depth;
                if (options.stats) options.stats->points += 1.0;
                if (pixel.x < 0.0 || pixel.x >= W || pixel.y < 0.0 || pixel.y >= H) continue;
                const int px = static_cast<int>(pixel.x);
                const int py = static_cast<int>(pixel.y);
                const int sx = std::min(sppSide - 1, static_cast<int>((pixel.x - px) * sppSide));
                const int sy = std::min(sppSide - 1, static_cast<int>((pixel.y - py) * sppSide));
                depthTest((static_cast<size_t>(py) * W + px) * spp + (sy * sppSide + sx),
                          d, p.pr.color, depth, color, options.stats);
            }
        }
        resolveSet(W, H, spp, depth, color, background, accum);
    }

    for (auto& c : accum) c /= static_cast<double>(std::max(1, numSets));
    return accum;
}

double psnr(const Image& a, const Image& b)
{
    const size_t n = std::min(a.size(), b.size());
    if (n == 0) return 0.0;
    double mse = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec3 d = a[i] - b[i];
        mse += glm::dot(d, d);
    }
    mse /= static_cast<double>(n * 3);
    if (mse <= 1e-20) return 120.0;
    return 10.0 * std::log10(1.0 / mse);
}

double ssim(const Image& a, const Image& b, int W, int H)
{
    // Mean SSIM over 8x8 windows (step 4) on luminance.
    auto lum = [](const glm::dvec3& c) { return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b; };
    const double C1 = 0.01 * 0.01;
    const double C2 = 0.03 * 0.03;

    double total = 0.0;
    int    count = 0;
    for (int y0 = 0; y0 + 8 <= H; y0 += 4) {
        for (int x0 = 0; x0 + 8 <= W; x0 += 4) {
            double ma = 0, mb = 0;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x) {
                    const size_t i = static_cast<size_t>(y0 + y) * W + (x0 + x);
                    ma += lum(a[i]);
                    mb += lum(b[i]);
                }
            ma /= 64.0; mb /= 64.0;
            double va = 0, vb = 0, cov = 0;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x) {
                    const size_t i = static_cast<size_t>(y0 + y) * W + (x0 + x);
                    const double da = lum(a[i]) - ma;
                    const double db = lum(b[i]) - mb;
                    va += da * da; vb += db * db; cov += da * db;
                }
            va /= 63.0; vb /= 63.0; cov /= 63.0;
            const double s = ((2 * ma * mb + C1) * (2 * cov + C2)) /
                             ((ma * ma + mb * mb + C1) * (va + vb + C2));
            total += s;
            ++count;
        }
    }
    return count ? total / count : 1.0;
}

} // namespace GSView::oracle
