#include "pch.h"

// Phase 1 of docs/todo/PLAN_footprint_aware_density_calibration.md: the unified
// footprint calibration N = densityScale * spp * A(v) * g(o) / a and its special
// cases, the C3+R radial keep probability, and variable-footprint point counts.
// Derivation: docs/paper/NOTE_footprint_density_calibration.md. No Vulkan.
#include "GaussianPointMath.h"

#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <vector>

using namespace GSView::gpm;

namespace {

constexpr double kPi = 3.14159265358979323846;
const double kOpacities[] = { 0.02, 0.2, 0.5, 0.8, 0.95, 0.999 };

// Camera at `eye` looking at `target` (camera +Z = forward, the PinholeCamera
// convention); world +Y is up.
PinholeCamera lookAtCamera(const glm::dvec3& eye, const glm::dvec3& target, double focal)
{
	const glm::dvec3 f = glm::normalize(target - eye);
	const glm::dvec3 r = glm::normalize(glm::cross(glm::dvec3(0, 1, 0), f));
	const glm::dvec3 u = glm::cross(f, r);
	PinholeCamera cam;
	cam.viewRot = glm::transpose(glm::dmat3(r, u, f));   // rows r, u, f: world -> camera
	cam.viewPos = eye;
	cam.focalX = cam.focalY = focal;
	cam.tanFovX = cam.tanFovY = 10.0;                     // no frustum clamp in these tests
	return cam;
}

double depthOf(const PinholeCamera& cam, const glm::dvec3& p) { return worldToCamera(cam, p).z; }

double footprintOf(const glm::dvec3& logScale, const glm::dquat& rot, const glm::dvec3& pos,
                   const PinholeCamera& cam)
{
	return projectedFootprintArea(covariance2D(covariance3D(logScale, rot), pos, cam, 0.0));
}

} // namespace

// o <= Li2(o) <= -log(1-o): the GPS term sits between Proportional and Extinction,
// which is why neither view-independent rule reproduces GPS at every opacity.
TEST(FootprintCalibration, OpacityTermsAreOrdered)
{
	for (double o : kOpacities) {
		const double p = opacityTerm(OpacityRule::Proportional, o);
		const double d = opacityTerm(OpacityRule::Dilog, o);
		const double e = opacityTerm(OpacityRule::Extinction, o);
		EXPECT_DOUBLE_EQ(p, o);
		EXPECT_NEAR(d, dilog(o), 1e-15);
		EXPECT_NEAR(e, -std::log(1.0 - o), 1e-12);
		EXPECT_LE(p, d);
		EXPECT_LE(d, e);
	}
	EXPECT_EQ(opacityTerm(OpacityRule::Extinction, 0.0), 0.0);
	EXPECT_EQ(opacityTerm(OpacityRule::Dilog, -1.0), 0.0);
	EXPECT_TRUE(std::isfinite(opacityTerm(OpacityRule::Extinction, 1.0)));
}

TEST(FootprintCalibration, ProjectedFootprintArea)
{
	EXPECT_NEAR(projectedFootprintArea(glm::dmat2(4.0, 0.0, 0.0, 9.0)), 2.0 * kPi * 6.0, 1e-12);
	EXPECT_NEAR(projectedFootprintArea(glm::dmat2(2.0, 1.0, 1.0, 2.0)), 2.0 * kPi * std::sqrt(3.0), 1e-12);
	EXPECT_EQ(projectedFootprintArea(glm::dmat2(1.0, 1.0, 1.0, 1.0)), 0.0);   // singular
	EXPECT_EQ(projectedFootprintArea(glm::dmat2(-1.0, 0.0, 0.0, 1.0)), 0.0);  // not SPD
}

// Each level reduces to the formula it names; C3 with Li2 is exactly the GPS E[N].
TEST(FootprintCalibration, LevelsReduceToExistingRules)
{
	CalibrationInputs in;
	in.objectDepth = 4.0; in.splatDepth = 2.0;
	in.focalX = in.focalY = 400.0; in.nearZ = 0.05; in.referencePixelLength = 0.01;
	const glm::dmat2 cov(9.0, 2.0, 2.0, 4.0);
	in.footprintArea = projectedFootprintArea(cov);
	in.spp = 4.0;
	const double baseK = 512.0, dS = 1.5, o = 0.6;
	const double g = -std::log(1.0 - o);

	EXPECT_NEAR(calibratedCount(CalibrationLevel::None, OpacityRule::Extinction, o, baseK, dS, in),
	            dS * g * baseK, 1e-9);
	EXPECT_NEAR(calibratedCount(CalibrationLevel::ObjectZoom, OpacityRule::Extinction, o, baseK, dS, in),
	            dS * g * baseK * pixelDensityScale(4.0, 400, 400, 0.01, 0.05), 1e-9);
	EXPECT_NEAR(calibratedCount(CalibrationLevel::PerSplatDepth, OpacityRule::Extinction, o, baseK, dS, in),
	            dS * g * baseK * pixelDensityScale(2.0, 400, 400, 0.01, 0.05), 1e-9);
	// C3 is absolute: baseK is ignored.
	EXPECT_NEAR(calibratedCount(CalibrationLevel::PerSplatFootprint, OpacityRule::Dilog, o, 1.0e9, 1.0, in),
	            in.spp * expectedPointCount(glm::determinant(cov), o), 1e-9);
	// A splat at the object centre makes C1 and C2 coincide.
	in.splatDepth = in.objectDepth;
	EXPECT_DOUBLE_EQ(
		calibratedCount(CalibrationLevel::ObjectZoom, OpacityRule::Proportional, o, baseK, dS, in),
		calibratedCount(CalibrationLevel::PerSplatDepth, OpacityRule::Proportional, o, baseK, dS, in));
	// Invalid inputs give zero, never a negative or non-finite count.
	EXPECT_EQ(calibratedCount(CalibrationLevel::None, OpacityRule::Extinction, 0.0, baseK, dS, in), 0.0);
	EXPECT_EQ(calibratedCount(CalibrationLevel::None, OpacityRule::Extinction, o, baseK, -1.0, in), 0.0);
	in.splatDepth = -1.0;
	EXPECT_EQ(calibratedCount(CalibrationLevel::PerSplatDepth, OpacityRule::Extinction, o, baseK, dS, in), 0.0);
}

// Which view changes each level responds to (plan Sec. 3.2):
//   orbit about the object centre -> C1 constant (z_c fixed),
//   rotating an anisotropic splat  -> C2 constant (z_i fixed), C3 changes.
TEST(FootprintCalibration, LevelInvariances)
{
	const glm::dvec3 centre(0.0);
	const glm::dvec3 disc(-3.0, -3.0, -6.0);   // log std-devs: a thin disc in the local XY plane
	const double focal = 400.0;

	std::vector<double> c1, c3;
	for (int k = 0; k < 12; ++k) {
		const double phi = k * kPi / 6.0;
		const PinholeCamera cam = lookAtCamera(glm::dvec3(4.0 * std::cos(phi), 0.0, 4.0 * std::sin(phi)),
		                                       centre, focal);
		CalibrationInputs in;
		in.objectDepth = depthOf(cam, centre);
		in.focalX = in.focalY = focal;
		c1.push_back(calibratedCount(CalibrationLevel::ObjectZoom, OpacityRule::Extinction, 0.5, 512.0, 1.0, in));
		c3.push_back(footprintOf(disc, glm::dquat(1, 0, 0, 0), centre, cam));
	}
	for (double v : c1) EXPECT_NEAR(v, c1.front(), 1e-9 * c1.front());
	// The disc (normal = world Z) is face-on at phi = 90 deg and edge-on at phi = 0.
	EXPECT_GT(c3[3] / c3[0], 10.0);

	const PinholeCamera cam = lookAtCamera(glm::dvec3(0.0, 0.0, 4.0), centre, focal);
	const glm::dvec3 splat(0.3, 0.2, 0.0);
	CalibrationInputs in;
	in.splatDepth = depthOf(cam, splat);
	in.focalX = in.focalY = focal;
	const double faceOn = footprintOf(disc, glm::dquat(1, 0, 0, 0), splat, cam);
	const glm::dquat tilt = glm::angleAxis(1.3, glm::dvec3(0, 1, 0));
	const double tilted = footprintOf(disc, tilt, splat, cam);
	// C2 only sees the splat's depth, which rotation does not move.
	const double c2 = calibratedCount(CalibrationLevel::PerSplatDepth, OpacityRule::Extinction, 0.5, 512.0, 1.0, in);
	EXPECT_GT(c2, 0.0);
	EXPECT_GT(faceOn / tilted, 3.0);
	in.footprintArea = faceOn;
	const double c3Face = calibratedCount(CalibrationLevel::PerSplatFootprint, OpacityRule::Dilog, 0.5, 1.0, 1.0, in);
	in.footprintArea = tilted;
	const double c3Tilt = calibratedCount(CalibrationLevel::PerSplatFootprint, OpacityRule::Dilog, 0.5, 1.0, 1.0, in);
	EXPECT_NEAR(c3Face / c3Tilt, faceOn / tilted, 1e-12 * faceOn / tilted);
}

TEST(FootprintCalibration, RadialKeepRangeAndLimits)
{
	for (double o : kOpacities) {
		EXPECT_NEAR(radialKeepProbability(o, 0.0), 1.0, 1e-12);
		double prev = 1.0;
		for (double r2 = 0.0; r2 <= 60.0; r2 += 0.25) {
			const double p = radialKeepProbability(o, r2);
			EXPECT_GT(p, 0.0);
			EXPECT_LE(p, 1.0);
			EXPECT_LE(p, prev + 1e-15);   // non-increasing in r
			prev = p;
		}
		// Far tail: Proportional / Extinction.
		EXPECT_NEAR(radialKeepProbability(o, 200.0), o / -std::log(1.0 - o), 1e-9);
	}
	EXPECT_NEAR(radialKeepProbability(1e-9, 3.0), 1.0, 1e-8);   // low opacity: nothing to correct
	EXPECT_EQ(radialKeepProbability(0.0, 3.0), 1.0);
	EXPECT_NEAR(radialKeepProbability(0.5, -1.0), 1.0, 1e-12);   // negative r2 treated as 0
}

// Extinction candidates x p(r) integrate to the GPS count 2*pi*Li2(o) (unit det).
TEST(FootprintCalibration, RadialKeepIntegratesToDilog)
{
	for (double o : kOpacities) {
		const double ext = -std::log(1.0 - o);
		// integral over the plane of p(r) * ext * exp(-r^2/2)  with u = r^2/2:
		// 2*pi * int_0^inf p(2u) * ext * exp(-u) du. Near-opaque splats put a
		// log-sharp feature of width ~(1-o) at u = 0, so [0, 0.1] gets a finer grid.
		auto simpson = [&](double a, double b, int n) {
			const double h = (b - a) / n;
			double sum = 0.0;
			for (int i = 0; i <= n; ++i) {
				const double u = a + i * h;
				const double f = radialKeepProbability(o, 2.0 * u) * ext * std::exp(-u);
				sum += f * ((i == 0 || i == n) ? 1.0 : (i % 2 ? 4.0 : 2.0));
			}
			return sum * h / 3.0;
		};
		const double integral = 2.0 * kPi * (simpson(0.0, 0.1, 200000) + simpson(0.1, 60.0, 60000));
		EXPECT_NEAR(integral / (2.0 * kPi * dilog(o)), 1.0, 1e-8) << "o=" << o;
	}
}

// Monte Carlo: Rayleigh radii (the projected-3D-Gaussian law) thinned by p(r) follow
// the corrected radial CDF F(r) = 1 - Li2(o e^{-r^2/2}) / Li2(o) that the GPS path
// samples directly, and the acceptance rate is Li2(o) / -log(1-o).
TEST(FootprintCalibration, RadialKeepReproducesCorrectedRadius)
{
	for (double o : { 0.3, 0.9, 0.99 }) {
		RandStream rng(particleSeed(SeedMode::Deterministic, 17u, static_cast<std::uint32_t>(o * 1000), 0u));
		const int trials = 200000;
		const double probes[] = { 0.5, 1.0, 1.5, 2.0, 3.0 };
		int below[5] = {};
		int kept = 0;
		for (int i = 0; i < trials; ++i) {
			const double r = std::sqrt(-2.0 * std::log(std::max(rng.next(), 1e-300)));
			if (rng.next() >= radialKeepProbability(o, r * r)) continue;
			++kept;
			for (int k = 0; k < 5; ++k) below[k] += r <= probes[k];
		}
		const double rate = static_cast<double>(kept) / trials;
		EXPECT_NEAR(rate, dilog(o) / -std::log(1.0 - o), 4.0 * std::sqrt(0.25 / trials)) << "o=" << o;
		for (int k = 0; k < 5; ++k) {
			const double r = probes[k];
			const double F = 1.0 - dilog(o * std::exp(-0.5 * r * r)) / dilog(o);
			EXPECT_NEAR(static_cast<double>(below[k]) / kept, F, 4.0 * std::sqrt(0.25 / kept))
				<< "o=" << o << " r=" << r;
		}
	}
}

// The C3+R pipeline end to end: Extinction-calibrated candidates at C3 times the
// mean keep probability equal the Li2-calibrated (GPS) count.
TEST(FootprintCalibration, ExtinctionCandidatesThinToGpsCount)
{
	CalibrationInputs in;
	in.footprintArea = projectedFootprintArea(glm::dmat2(6.0, 1.0, 1.0, 3.0));
	in.spp = 4.0;
	for (double o : kOpacities) {
		const double candidates = calibratedCount(CalibrationLevel::PerSplatFootprint, OpacityRule::Extinction,
		                                          o, 1.0, 1.0, in);
		const double gps = calibratedCount(CalibrationLevel::PerSplatFootprint, OpacityRule::Dilog, o, 1.0, 1.0, in);
		EXPECT_NEAR(candidates * (dilog(o) / -std::log(1.0 - o)), gps, 1e-9 * gps);
		EXPECT_GE(candidates, gps);
	}
}

TEST(FootprintCalibration, VariableFootprintCounts)
{
	EXPECT_DOUBLE_EQ(footprintPointCount(100.0, 1), 100.0);
	EXPECT_DOUBLE_EQ(footprintPointCount(100.0, 2), 25.0);
	EXPECT_DOUBLE_EQ(footprintPointCount(90.0, 3), 10.0);
	EXPECT_DOUBLE_EQ(footprintPointCount(100.0, 0), 100.0);   // s < 1 behaves as 1
	EXPECT_EQ(footprintPointCount(-5.0, 2), 0.0);

	EXPECT_EQ(adaptiveFootprint(10.0, 0.0, 4), 1);    // disabled
	EXPECT_EQ(adaptiveFootprint(10.0, 0.25, 4), 2);   // floor(2.5)
	EXPECT_EQ(adaptiveFootprint(10.0, 1.0, 4), 4);    // clamped to sMax
	EXPECT_EQ(adaptiveFootprint(0.5, 1.0, 4), 1);     // sub-subpixel splat keeps unit points
	EXPECT_EQ(adaptiveFootprint(10.0, 1.0, 0), 1);    // sMax < 1 behaves as 1
	for (double sigma = 0.5; sigma < 40.0; sigma += 0.5) {
		const int s = adaptiveFootprint(sigma, 0.3, 8);
		EXPECT_GE(s, 1);
		EXPECT_LE(s, 8);
		if (s > 1) EXPECT_LE(s / sigma, 0.3 + 1e-12);   // relative blur bounded by kappa
	}
}
