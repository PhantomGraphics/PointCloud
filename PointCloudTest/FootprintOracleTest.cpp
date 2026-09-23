#include "pch.h"

// Phase 2 of docs/todo/PLAN_footprint_aware_density_calibration.md: CPU-oracle
// checks of the footprint calibration before any GPU work.
//   * renderMonteCarlo() with variable-footprint points (level F): s = 1 is
//     bit-identical to the GPS port, larger s trades bias for ~1/s^2 points.
//   * renderParticles3D(): 3D particles with C3+R converge to the analytic GPS
//     image; C3 without the radial correction does not at high opacity; the two
//     residuals left after C3+R (exact perspective, per-particle depth) are
//     isolated by the ablation switches.
// Measured values are printed ("[footprint]") and recorded in the plan.
#include "GaussianPointOracle.h"

#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

using namespace GSView::oracle;
namespace gpm = GSView::gpm;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int    kW = 96, kH = 96;
constexpr int    kSppSide = 2;
constexpr int    kSets = 300;
const glm::dvec3 kBg(0.05, 0.05, 0.08);

OracleCamera frontalCamera()
{
	OracleCamera c;
	c.viewRot = glm::dmat3(1.0);
	c.viewPos = glm::dvec3(0.0);
	c.focalX = c.focalY = 200.0;
	return c;
}

Gaussian3D splat(glm::dvec3 pos, glm::dvec3 sigma, double opacity, glm::dvec3 color)
{
	Gaussian3D g;
	g.pos = pos;
	g.logScale = glm::log(sigma);
	g.opacity = opacity;
	g.color = color;
	return g;
}

// Exact C3+R: the configuration the theory says converges to the analytic image.
ParticleOptions exactC3R()
{
	ParticleOptions o;
	o.level = gpm::CalibrationLevel::PerSplatFootprint;
	o.rule = gpm::OpacityRule::Extinction;
	o.radialCorrection = true;
	o.centreDepth = true;
	o.linearizedProjection = true;
	return o;
}

double gpsExpectedPerSet(const Gaussian3D& g, const OracleCamera& cam)
{
	gpm::PinholeCamera pin;
	pin.viewRot = cam.viewRot; pin.viewPos = cam.viewPos;
	pin.focalX = cam.focalX; pin.focalY = cam.focalY;
	pin.tanFovX = pin.tanFovY = 4.0;
	const glm::dmat2 cov = gpm::covariance2D(gpm::covariance3D(g.logScale, g.rot), g.pos, pin, cam.lowPass);
	return gpm::expectedPointCount(glm::determinant(cov), g.opacity) * kSppSide * kSppSide;
}

} // namespace

TEST(FootprintOracle, DefaultOptionsAreBitIdenticalToGpsPort)
{
	const std::vector<Gaussian3D> scene = {
		splat({ 0.1, -0.05, 6.0 }, { 0.2, 0.1, 0.05 }, 0.7, { 0.9, 0.4, 0.2 }),
		splat({ -0.2, 0.1, 8.0 }, { 0.3, 0.3, 0.3 }, 0.4, { 0.2, 0.6, 0.9 }) };
	const Image a = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, 20, kBg, 7u);
	RenderStats stats;
	MonteCarloOptions opt;
	opt.stats = &stats;
	const Image b = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, 20, kBg, 7u, opt);
	ASSERT_EQ(a.size(), b.size());
	for (size_t i = 0; i < a.size(); ++i) ASSERT_EQ(a[i], b[i]) << "pixel " << i;
	EXPECT_GT(stats.points, 0.0);
	EXPECT_LE(stats.subpixelWrites, stats.points);   // s = 1: at most one write per point
}

// C3+R with the GPS depth rule and the EWA linearisation is the GPS image in
// expectation, with the GPS point count.
TEST(FootprintOracle, ParticlesC3RConvergeToAnalytic)
{
	for (double o : { 0.5, 0.9 }) {
		const Gaussian3D g = splat({ 0.0, 0.0, 6.0 }, { 0.15, 0.15, 0.15 }, o, { 0.9, 0.4, 0.2 });
		const std::vector<Gaussian3D> scene = { g };
		const Image ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
		RenderStats stats;
		ParticleOptions opt = exactC3R();
		opt.stats = &stats;
		const Image img = renderParticles3D(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 3u, opt);
		const double p = psnr(img, ref), s = ssim(img, ref, kW, kH);
		const double perSet = stats.points / kSets;
		const double target = gpsExpectedPerSet(g, frontalCamera());
		std::printf("[footprint] C3+R o=%.1f  PSNR=%.2f dB  SSIM=%.4f  points/set=%.1f (GPS %.1f)\n",
		            o, p, s, perSet, target);
		EXPECT_GE(p, 35.0) << "o=" << o;
		EXPECT_GE(s, 0.98) << "o=" << o;
		EXPECT_NEAR(perSet / target, 1.0, 0.02) << "o=" << o;
	}
}

// Without the radial correction (ViewConditioned's constant keep probability, i.e.
// the right count with a Gaussian profile) the image is biased at high opacity.
TEST(FootprintOracle, RadialCorrectionRemovesHighOpacityBias)
{
	const std::vector<Gaussian3D> scene = {
		splat({ 0.0, 0.0, 6.0 }, { 0.15, 0.15, 0.15 }, 0.95, { 0.9, 0.4, 0.2 }) };
	const Image ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);

	ParticleOptions corrected = exactC3R();
	ParticleOptions plain = exactC3R();
	plain.radialCorrection = false;
	plain.rule = gpm::OpacityRule::Dilog;   // same expected count as GPS, Gaussian profile

	const double pc = psnr(renderParticles3D(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 5u, corrected), ref);
	const double pp = psnr(renderParticles3D(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 5u, plain), ref);
	std::printf("[footprint] o=0.95  C3 (no radial) PSNR=%.2f dB   C3+R PSNR=%.2f dB\n", pp, pc);
	EXPECT_GE(pc, 35.0);
	EXPECT_GT(pc, pp + 5.0);
}

// Residual (a): exact perspective vs the EWA linearisation. Negligible for a small
// distant splat, measurable for a large near one.
TEST(FootprintOracle, PerspectiveResidualGrowsWithApparentSize)
{
	// Half-resolution image and focal length (same field of view) keep the large splat's
	// point count affordable in Debug.
	auto residual = [](const Gaussian3D& g) {
		const int w = kW / 2, h = kH / 2;
		OracleCamera cam = frontalCamera();
		cam.focalX = cam.focalY = 100.0;
		const std::vector<Gaussian3D> scene = { g };
		const Image ref = renderAnalytic(scene, cam, w, h, kBg);
		ParticleOptions lin = exactC3R();
		ParticleOptions exact = exactC3R();
		exact.linearizedProjection = false;
		const double pl = psnr(renderParticles3D(scene, cam, w, h, kSppSide, kSets, kBg, 9u, lin), ref);
		const double pe = psnr(renderParticles3D(scene, cam, w, h, kSppSide, kSets, kBg, 9u, exact), ref);
		return std::make_pair(pl, pe);
	};
	// sigma/depth = 0.025 vs 0.2, off-axis so the perspective Jacobian has a depth term.
	const auto far = residual(splat({ 0.3, 0.2, 6.0 }, { 0.15, 0.15, 0.15 }, 0.7, { 0.9, 0.4, 0.2 }));
	const auto nearBig = residual(splat({ 0.06, 0.04, 1.2 }, { 0.24, 0.24, 0.24 }, 0.7, { 0.9, 0.4, 0.2 }));
	std::printf("[footprint] perspective residual: small/far  lin=%.2f exact=%.2f dB;  large/near  lin=%.2f exact=%.2f dB\n",
	            far.first, far.second, nearBig.first, nearBig.second);
	EXPECT_GE(far.second, 35.0);
	EXPECT_LT(nearBig.second, far.second);
	EXPECT_LT(nearBig.second, nearBig.first);
}

// Residual (b): per-particle depth vs the GPS splat-centre depth. Two splats that
// interpenetrate in depth composite differently.
TEST(FootprintOracle, DepthRuleResidualForInterpenetratingSplats)
{
	const std::vector<Gaussian3D> scene = {
		splat({ -0.05, 0.0, 5.9 }, { 0.15, 0.15, 0.8 }, 0.8, { 0.9, 0.2, 0.2 }),
		splat({ 0.05, 0.0, 6.1 }, { 0.15, 0.15, 0.8 }, 0.8, { 0.2, 0.8, 0.3 }) };
	const Image ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
	ParticleOptions centre = exactC3R();
	ParticleOptions own = exactC3R();
	own.centreDepth = false;
	const double pc = psnr(renderParticles3D(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 11u, centre), ref);
	const double po = psnr(renderParticles3D(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 11u, own), ref);
	std::printf("[footprint] depth rule: centre depth PSNR=%.2f dB   per-particle depth PSNR=%.2f dB\n", pc, po);
	EXPECT_GE(pc, 35.0);
	EXPECT_LT(po, pc - 3.0);
}

// Level F: s x s footprint points cut the point count by ~s^2 at a bias that grows
// with s / sigma; the adaptive rule and blur compensation limit that bias.
TEST(FootprintOracle, VariableFootprintTradesPointsForBias)
{
	// sigma ~ 5 px = 10 subpixels (large) and ~2 px = 4 subpixels (small).
	const std::vector<Gaussian3D> scene = {
		splat({ -0.35, 0.0, 6.0 }, { 0.15, 0.15, 0.15 }, 0.8, { 0.9, 0.4, 0.2 }),
		splat({ 0.4, 0.2, 6.0 }, { 0.06, 0.06, 0.06 }, 0.8, { 0.2, 0.6, 0.9 }) };
	const Image ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);

	auto run = [&](MonteCarloOptions opt) {
		RenderStats stats;
		opt.stats = &stats;
		const Image img = renderMonteCarlo(scene, frontalCamera(), kW, kH, kSppSide, kSets, kBg, 13u, opt);
		return std::make_pair(psnr(img, ref), stats);
	};
	MonteCarloOptions base;
	const auto r1 = run(base);
	std::printf("[footprint] F s=1: PSNR=%.2f dB points/set=%.0f writes/set=%.0f\n",
	            r1.first, r1.second.points / kSets, r1.second.subpixelWrites / kSets);
	double prev = r1.first;
	for (int s : { 2, 4 }) {
		MonteCarloOptions opt;
		opt.footprintSubpixels = s;
		const auto r = run(opt);
		opt.compensateBlur = true;
		const auto rc = run(opt);
		std::printf("[footprint] F s=%d: PSNR=%.2f dB (compensated %.2f) points/set=%.0f (x%.3f) writes/set=%.0f\n",
		            s, r.first, rc.first, r.second.points / kSets, r.second.points / r1.second.points,
		            r.second.subpixelWrites / kSets);
		EXPECT_NEAR(r.second.points / r1.second.points, 1.0 / (s * s), 0.1 / (s * s));
		EXPECT_LT(r.first, prev + 0.5);   // bias does not shrink as the footprint grows
		EXPECT_GE(rc.first, r.first);     // compensation never hurts here
		prev = r.first;
	}
	MonteCarloOptions adaptive;
	adaptive.adaptiveKappa = 0.4;
	adaptive.footprintMax = 4;
	const auto ra = run(adaptive);
	std::printf("[footprint] F adaptive kappa=0.4: PSNR=%.2f dB points/set=%.0f (x%.3f)\n",
	            ra.first, ra.second.points / kSets, ra.second.points / r1.second.points);
	EXPECT_LT(ra.second.points, 0.5 * r1.second.points);
	EXPECT_GE(ra.first, 35.0);
}

// Measurement only (run with --gtest_also_run_disabled_tests, Release): separates
// bias from Monte Carlo variance. PSNR against the analytic image is noise-limited
// for large splats and large footprints; if an error is pure variance, doubling
// the sample sets raises PSNR by ~3 dB, while a bias leaves it flat. Also reports
// the GPS port itself as the noise floor for the perspective residual.
TEST(FootprintOracle, DISABLED_BiasVersusVariance)
{
	auto report = [](const char* label, const std::vector<Gaussian3D>& scene, auto render) {
		const Image ref = renderAnalytic(scene, frontalCamera(), kW, kH, kBg);
		const double p1 = psnr(render(1000), ref);
		const double p4 = psnr(render(4000), ref);
		std::printf("[footprint] %-34s PSNR 1000 sets=%.2f  4000 sets=%.2f  (pure variance: +6.0 dB)\n",
		            label, p1, p4);
	};
	const std::vector<Gaussian3D> nearBig = {
		splat({ 0.06, 0.04, 1.2 }, { 0.24, 0.24, 0.24 }, 0.7, { 0.9, 0.4, 0.2 }) };
	report("near/large GPS port", nearBig, [&](int n) {
		return renderMonteCarlo(nearBig, frontalCamera(), kW, kH, kSppSide, n, kBg, 21u); });
	report("near/large C3+R linearised", nearBig, [&](int n) {
		return renderParticles3D(nearBig, frontalCamera(), kW, kH, kSppSide, n, kBg, 21u, exactC3R()); });
	report("near/large C3+R exact perspective", nearBig, [&](int n) {
		ParticleOptions o = exactC3R(); o.linearizedProjection = false;
		return renderParticles3D(nearBig, frontalCamera(), kW, kH, kSppSide, n, kBg, 21u, o); });

	const std::vector<Gaussian3D> pair = {
		splat({ -0.35, 0.0, 6.0 }, { 0.15, 0.15, 0.15 }, 0.8, { 0.9, 0.4, 0.2 }),
		splat({ 0.4, 0.2, 6.0 }, { 0.06, 0.06, 0.06 }, 0.8, { 0.2, 0.6, 0.9 }) };
	for (int s : { 1, 2, 4 }) {
		for (bool comp : { false, true }) {
			if (s == 1 && comp) continue;
			char label[64];
			std::snprintf(label, sizeof(label), "F s=%d%s", s, comp ? " compensated" : "");
			report(label, pair, [&](int n) {
				MonteCarloOptions o; o.footprintSubpixels = s; o.compensateBlur = comp;
				return renderMonteCarlo(pair, frontalCamera(), kW, kH, kSppSide, n, kBg, 23u, o); });
		}
	}
	report("F adaptive kappa=0.4", pair, [&](int n) {
		MonteCarloOptions o; o.adaptiveKappa = 0.4; o.footprintMax = 4;
		return renderMonteCarlo(pair, frontalCamera(), kW, kH, kSppSide, n, kBg, 23u, o); });
}
