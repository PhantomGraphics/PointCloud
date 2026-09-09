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

    Image accum(static_cast<size_t>(W) * H, glm::dvec3(0.0));

    std::vector<double>     depth(nSub);
    std::vector<glm::dvec3> color(nSub);

    for (int s = 0; s < numSets; ++s) {
        std::fill(depth.begin(), depth.end(), 1e300);
        std::fill(color.begin(), color.end(), glm::dvec3(0.0));

        for (std::uint32_t gid = 0; gid < proj.size(); ++gid) {
            const Projected& pr = proj[gid];

            const double EN = gpm::expectedPointCount(pr.detCov2d, pr.opacity)
                              * static_cast<double>(spp);
            if (EN <= 0.0) continue;

            const std::uint32_t seed = gpm::particleSeed(
                gpm::SeedMode::FrameVarying, gid, 0u,
                baseSeed + static_cast<std::uint32_t>(s));
            gpm::RandStream rng(seed);
            const int N = gpm::poissonSample(EN, rng);

            for (int i = 0; i < N; ++i) {
                const glm::dvec2 off = gpm::sampleCorrectedOffset(pr.opacity, pr.cov2d, rng);
                const double fx = pr.mean.x + off.x;
                const double fy = pr.mean.y + off.y;
                if (fx < 0.0 || fx >= W || fy < 0.0 || fy >= H) continue;

                const int px = static_cast<int>(fx);
                const int py = static_cast<int>(fy);
                const int sx = std::min(sppSide - 1,
                    static_cast<int>((fx - px) * sppSide));
                const int sy = std::min(sppSide - 1,
                    static_cast<int>((fy - py) * sppSide));
                const size_t idx =
                    (static_cast<size_t>(py) * W + px) * spp + (sy * sppSide + sx);

                if (pr.depth < depth[idx]) {
                    depth[idx] = pr.depth;
                    color[idx] = pr.color;
                }
            }
        }

        // resolve this sample set
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

    for (auto& c : accum) c /= static_cast<double>(numSets);
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
