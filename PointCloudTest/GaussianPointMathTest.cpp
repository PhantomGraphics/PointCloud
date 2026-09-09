#include "pch.h"

// Phase 1 of docs/todo/PLAN_gsview_gaussian_point_pbvr.md: the pure CPU maths
// core and the analytic per-pixel coverage oracle. No Vulkan.
#include "GaussianPointMath.h"
#include "GaussianPointTestVectors.h"

#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace GSView::gpm;
namespace vec = GSView::gpm::vectors;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Numerically integrate lambda(x) = -log(1 - o*exp(-1/2 d^T C^-1 d)) over a wide
// grid, as an independent check of the closed form E[N] = 2*pi*sqrt(detC)*Li2(o).
double integrateLambda(const glm::dmat2& cov, double o)
{
	const glm::dmat2 inv = glm::inverse(cov);
	const double span = 6.0 * std::sqrt(std::max(cov[0][0], cov[1][1]));
	const int    n = 400;
	const double h = (2.0 * span) / n;
	double sum = 0.0;
	for (int iy = 0; iy < n; ++iy) {
		for (int ix = 0; ix < n; ++ix) {
			const glm::dvec2 d(-span + (ix + 0.5) * h, -span + (iy + 0.5) * h);
			const double m = glm::dot(d, inv * d);
			const double a = o * std::exp(-0.5 * m);
			sum += -std::log(1.0 - a);
		}
	}
	return sum * h * h;
}

} // namespace

// ---------------------------------------------------------------------------
// Scalar activations + shared vectors
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, SigmoidMatchesTestVectors)
{
	for (const auto& c : vec::kSigmoid)
		EXPECT_NEAR(sigmoid(c.x), c.s, 1e-12) << "x=" << c.x;
}

TEST(GaussianPointMath, ShDcToColorClampsToUnitRange)
{
	EXPECT_NEAR(shDcToColor(0.0), 0.5, 1e-12);
	EXPECT_DOUBLE_EQ(shDcToColor(1000.0), 1.0);
	EXPECT_DOUBLE_EQ(shDcToColor(-1000.0), 0.0);
}

// ---------------------------------------------------------------------------
// Dilogarithm
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, DilogMatchesTestVectors)
{
	for (const auto& c : vec::kDilog)
		EXPECT_NEAR(dilog(c.x), c.li2, 1e-11) << "x=" << c.x;
}

TEST(GaussianPointMath, DilogClampsOutsideUnitInterval)
{
	EXPECT_DOUBLE_EQ(dilog(1.5), kDilogAtOne);
	EXPECT_DOUBLE_EQ(dilog(-3.0), -kDilogAtOne / 2.0);
}

TEST(GaussianPointMath, InvDilogRoundTrips)
{
	for (double y = 0.02; y < 0.999; y += 0.03) {
		const double back = invDilog(dilog(y));
		EXPECT_NEAR(back, y, 1e-9) << "y=" << y;
	}
	EXPECT_NEAR(invDilog(0.0), 0.0, 1e-12);
	EXPECT_NEAR(invDilog(kDilogAtOne), 1.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Covariance
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, Covariance3DIdentityRotationIsDiagonalScaleSquared)
{
	const glm::dvec3 logS(std::log(2.0), std::log(3.0), std::log(0.5));
	const glm::dmat3 C = covariance3D(logS, glm::dquat(1.0, 0.0, 0.0, 0.0));
	EXPECT_NEAR(C[0][0], 4.0, 1e-12);
	EXPECT_NEAR(C[1][1], 9.0, 1e-12);
	EXPECT_NEAR(C[2][2], 0.25, 1e-12);
	EXPECT_NEAR(C[0][1], 0.0, 1e-12);
	EXPECT_NEAR(C[0][2], 0.0, 1e-12);
	EXPECT_NEAR(C[1][2], 0.0, 1e-12);
}

TEST(GaussianPointMath, Covariance3DIsSymmetricUnderRotation)
{
	const glm::dquat q = glm::normalize(glm::dquat(0.3, 0.5, -0.2, 0.7));
	const glm::dmat3 C = covariance3D(glm::dvec3(0.1, -0.4, 0.2), q);
	EXPECT_NEAR(C[0][1], C[1][0], 1e-12);
	EXPECT_NEAR(C[0][2], C[2][0], 1e-12);
	EXPECT_NEAR(C[1][2], C[2][1], 1e-12);
	// Trace is rotation-invariant: sum exp(2*logScale).
	const double tr = C[0][0] + C[1][1] + C[2][2];
	const double expTr = std::exp(0.2) + std::exp(-0.8) + std::exp(0.4);
	EXPECT_NEAR(tr, expTr, 1e-10);
}

TEST(GaussianPointMath, Covariance2DAnalyticIsotropicCase)
{
	// Camera at world (0,0,5) looking toward -Z; isotropic 3D covariance.
	PinholeCamera cam;
	cam.viewRot = glm::dmat3(-1, 0, 0,  0, 1, 0,  0, 0, -1);
	cam.viewPos = glm::dvec3(0.0, 0.0, 5.0);
	cam.focalX = cam.focalY = 100.0;
	cam.tanFovX = cam.tanFovY = 1.0;

	const double sigma2 = 0.04;
	glm::dmat3 cov3d(0.0);
	cov3d[0][0] = cov3d[1][1] = cov3d[2][2] = sigma2;

	const glm::dmat2 c2 = covariance2D(cov3d, glm::dvec3(0.0), cam, 0.3);

	// sigma2 * f^2 / z^2 + lowPass = 0.04 * 10000 / 25 + 0.3 = 16.3
	EXPECT_NEAR(c2[0][0], 16.3, 1e-9);
	EXPECT_NEAR(c2[1][1], 16.3, 1e-9);
	EXPECT_NEAR(c2[0][1], 0.0, 1e-9);
	EXPECT_NEAR(c2[1][0], 0.0, 1e-9);
}

TEST(GaussianPointMath, Covariance2DShrinksWithDistance)
{
	PinholeCamera cam;
	cam.viewRot = glm::dmat3(-1, 0, 0,  0, 1, 0,  0, 0, -1);
	cam.focalX = cam.focalY = 200.0;
	cam.tanFovX = cam.tanFovY = 1.0;

	glm::dmat3 cov3d(0.0);
	cov3d[0][0] = cov3d[1][1] = cov3d[2][2] = 0.01;

	cam.viewPos = glm::dvec3(0.0, 0.0, 3.0);
	const double near = covariance2D(cov3d, glm::dvec3(0.0), cam, 0.0)[0][0];
	cam.viewPos = glm::dvec3(0.0, 0.0, 9.0);
	const double far = covariance2D(cov3d, glm::dvec3(0.0), cam, 0.0)[0][0];

	// 1/z^2 falloff: (3/9)^2 = 1/9.
	EXPECT_NEAR(far / near, 1.0 / 9.0, 1e-9);
}

TEST(GaussianPointMath, Cholesky2DReconstructsMatrix)
{
	const glm::dmat2 A(4.0, 1.0, 1.0, 3.0);
	const glm::dmat2 L = cholesky2D(A);
	const glm::dmat2 rebuilt = L * glm::transpose(L);
	EXPECT_NEAR(rebuilt[0][0], A[0][0], 1e-12);
	EXPECT_NEAR(rebuilt[0][1], A[0][1], 1e-12);
	EXPECT_NEAR(rebuilt[1][0], A[1][0], 1e-12);
	EXPECT_NEAR(rebuilt[1][1], A[1][1], 1e-12);
}

// ---------------------------------------------------------------------------
// Expected point count
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, ExpectedPointCountMatchesTestVectors)
{
	for (const auto& c : vec::kExpectedCount) {
		const double got = expectedPointCount(c.cov2dIsotropic * c.cov2dIsotropic, c.opacity);
		EXPECT_NEAR(got, c.expectedN, 1e-9)
			<< "s2=" << c.cov2dIsotropic << " o=" << c.opacity;
	}
}

TEST(GaussianPointMath, ExpectedPointCountMatchesNumericIntegral)
{
	const glm::dmat2 cov(6.0, 1.5, 1.5, 4.0);
	for (double o : {0.2, 0.5, 0.9}) {
		const double closed = expectedPointCount(glm::determinant(cov), o);
		const double numeric = integrateLambda(cov, o);
		EXPECT_NEAR(closed, numeric, 0.01 * closed) << "o=" << o;
	}
}

TEST(GaussianPointMath, ExpectedPointCountIsZeroForDegenerateInputs)
{
	EXPECT_DOUBLE_EQ(expectedPointCount(0.0, 0.9), 0.0);
	EXPECT_DOUBLE_EQ(expectedPointCount(4.0, 0.0), 0.0);
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, PcgHashMatchesShaderConstants)
{
	// Same algorithm as gs_pbvr_gen.comp's pcg(); these are fixed reference values.
	EXPECT_EQ(pcgHash(0u), 129708002u);
	EXPECT_EQ(pcgHash(1u), 2831084092u);
}

TEST(GaussianPointMath, SeedModesBehaveAsDocumented)
{
	// Deterministic: independent of frame index.
	EXPECT_EQ(particleSeed(SeedMode::Deterministic, 7, 3, 0),
	          particleSeed(SeedMode::Deterministic, 7, 3, 999));
	// FrameVarying: different frames -> different streams.
	EXPECT_NE(particleSeed(SeedMode::FrameVarying, 7, 3, 0),
	          particleSeed(SeedMode::FrameVarying, 7, 3, 1));
	// Distinct particles -> distinct seeds.
	EXPECT_NE(particleSeed(SeedMode::Deterministic, 7, 3, 0),
	          particleSeed(SeedMode::Deterministic, 7, 4, 0));
}

TEST(GaussianPointMath, RandStreamIsReproducibleAndRoughlyUniform)
{
	RandStream a(12345u), b(12345u);
	double sum = 0.0;
	int    bins[10] = {};
	const int N = 40000;
	for (int i = 0; i < N; ++i) {
		const double va = a.next();
		EXPECT_DOUBLE_EQ(va, b.next());
		EXPECT_GE(va, 0.0);
		EXPECT_LT(va, 1.0);
		sum += va;
		bins[std::min(9, static_cast<int>(va * 10))]++;
	}
	EXPECT_NEAR(sum / N, 0.5, 0.01);
	for (int k = 0; k < 10; ++k)
		EXPECT_NEAR(bins[k] / static_cast<double>(N), 0.1, 0.015) << "bin " << k;
}

// Completion criterion: "乱数平均の点数が理論値の 3 標準誤差内に入る".
TEST(GaussianPointMath, PoissonSampleMeanIsWithinThreeStandardErrors)
{
	RandStream rng(999u);
	for (double lambda : {0.7, 4.0, 12.0, 45.0}) {
		const int T = 60000;
		double s = 0.0;
		for (int i = 0; i < T; ++i) s += poissonSample(lambda, rng);
		const double mean = s / T;
		const double se = std::sqrt(lambda / T); // Var(Poisson)=lambda
		EXPECT_LT(std::abs(mean - lambda), 3.0 * se) << "lambda=" << lambda;
	}
}

TEST(GaussianPointMath, CorrectedRadiusEmpiricalCdfMatchesClosedForm)
{
	const double o = 0.85;
	const double D = dilog(o);
	auto cdf = [&](double r) { return 1.0 - dilog(o * std::exp(-0.5 * r * r)) / D; };

	RandStream rng(2024u);
	const int M = 40000;
	std::vector<double> r(M);
	for (int i = 0; i < M; ++i) r[i] = sampleCorrectedRadius(o, rng.next());

	for (double rr : {0.5, 1.0, 1.5, 2.5}) {
		int below = 0;
		for (double v : r) if (v <= rr) ++below;
		const double emp = below / static_cast<double>(M);
		const double p = cdf(rr);
		const double se = std::sqrt(p * (1.0 - p) / M);
		EXPECT_LT(std::abs(emp - p), 4.0 * se) << "r=" << rr << " expected " << p;
	}
}

// ---------------------------------------------------------------------------
// Coverage oracle
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, ExpectedCoverageEqualsAlphaForUnitPixel)
{
	for (double a : {0.05, 0.3, 0.7, 0.95})
		EXPECT_NEAR(expectedCoverage(a, 1.0), a, 1e-12);
	// Sub-pixel area -> lower coverage per sample.
	EXPECT_LT(expectedCoverage(0.7, 0.25), 0.7);
}

TEST(GaussianPointMath, OverCompositesConsistentlyInBothAlphaModes)
{
	const glm::dvec4 srcS(1.0, 0.0, 0.0, 0.5); // straight red, alpha .5
	const glm::dvec4 dstS(0.0, 0.0, 1.0, 1.0); // straight opaque blue
	const glm::dvec4 outS = over(srcS, dstS, AlphaMode::Straight);

	const glm::dvec4 srcP(0.5, 0.0, 0.0, 0.5); // same, premultiplied
	const glm::dvec4 dstP(0.0, 0.0, 1.0, 1.0);
	const glm::dvec4 outP = over(srcP, dstP, AlphaMode::Premultiplied);

	EXPECT_NEAR(outS.a, 1.0, 1e-12);
	EXPECT_NEAR(outP.a, 1.0, 1e-12);
	// straight rgb == premultiplied rgb / alpha
	EXPECT_NEAR(outS.r, outP.r / outP.a, 1e-12);
	EXPECT_NEAR(outS.b, outP.b / outP.a, 1e-12);
	// half red over blue: 0.5 red + 0.5 blue
	EXPECT_NEAR(outS.r, 0.5, 1e-12);
	EXPECT_NEAR(outS.b, 0.5, 1e-12);
}

TEST(GaussianPointMath, CompositeExpectedMatchesHandCalculationForTwoLayers)
{
	Gaussian2D front;
	front.mean = glm::dvec2(0.0);
	front.cov = glm::dmat2(4.0, 0.0, 0.0, 4.0);
	front.opacity = 0.6;
	front.color = glm::dvec3(1.0, 0.0, 0.0);
	front.depth = 1.0;

	Gaussian2D back = front;
	back.opacity = 0.8;
	back.color = glm::dvec3(0.0, 1.0, 0.0);
	back.depth = 5.0;

	const glm::dvec2 p(0.0, 0.0);
	const glm::dvec3 bg(0.0, 0.0, 0.1);

	const double af = alphaAt(front, p); // = 0.6 at the mean
	const double ab = alphaAt(back, p);  // = 0.8
	const glm::dvec3 expected =
		af * front.color +
		(1.0 - af) * (ab * back.color + (1.0 - ab) * bg);

	// Order should not matter (compositeExpected sorts by depth).
	const glm::dvec3 got = compositeExpected({back, front}, p, bg, 1.0);
	EXPECT_NEAR(got.r, expected.r, 1e-12);
	EXPECT_NEAR(got.g, expected.g, 1e-12);
	EXPECT_NEAR(got.b, expected.b, 1e-12);
}

// Completion criterion: single-Gaussian average coverage matches the oracle
// across a spread of parameters (here: opacity and screen-space scale).
TEST(GaussianPointMath, MonteCarloSingleGaussianCoverageMatchesOracle)
{
	struct Cfg { double sigma; double opacity; };
	for (Cfg cfg : std::vector<Cfg>{ {3.0, 0.8}, {5.0, 0.5}, {4.0, 0.95} }) {
		Gaussian2D g;
		g.mean = glm::dvec2(0.0);
		g.cov = glm::dmat2(cfg.sigma * cfg.sigma, 0.0, 0.0, cfg.sigma * cfg.sigma);
		g.opacity = cfg.opacity;

		const double EN = expectedPointCount(glm::determinant(g.cov), g.opacity);

		const int T = 4000;
		const glm::dvec2 probes[] = { {0.0, 0.0}, {cfg.sigma, 0.0}, {0.0, 2.0 * cfg.sigma} };
		int hits[3] = {};

		RandStream rng(4242u + static_cast<std::uint32_t>(cfg.sigma * 10));
		for (int t = 0; t < T; ++t) {
			const int n = poissonSample(EN, rng);
			bool covered[3] = { false, false, false };
			for (int i = 0; i < n; ++i) {
				const glm::dvec2 pt = g.mean + sampleCorrectedOffset(g.opacity, g.cov, rng);
				for (int k = 0; k < 3; ++k) {
					const glm::dvec2 d = pt - probes[k];
					if (std::abs(d.x) <= 0.5 && std::abs(d.y) <= 0.5) covered[k] = true;
				}
			}
			for (int k = 0; k < 3; ++k) if (covered[k]) hits[k]++;
		}

		for (int k = 0; k < 3; ++k) {
			const double emp = hits[k] / static_cast<double>(T);
			const double oracle = expectedCoverage(alphaAt(g, probes[k]), 1.0);
			const double se = std::sqrt(std::max(oracle * (1.0 - oracle), 1e-6) / T);
			EXPECT_LT(std::abs(emp - oracle), 4.0 * se)
				<< "sigma=" << cfg.sigma << " o=" << cfg.opacity
				<< " probe=" << k << " emp=" << emp << " oracle=" << oracle;
		}
	}
}

// ---------------------------------------------------------------------------
// Spherical harmonics
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, EvalSHDegreeZeroIsDcTimesC0PlusHalf)
{
	const glm::dvec3 dc(0.4, -0.2, 1.3);
	const glm::dvec3 got = evalSH(0, dc, nullptr, glm::dvec3(0, 0, 1));
	EXPECT_NEAR(got.r, kShC0 * 0.4 + 0.5, 1e-12);
	EXPECT_NEAR(got.g, std::max(kShC0 * -0.2 + 0.5, 0.0), 1e-12);
	EXPECT_NEAR(got.b, kShC0 * 1.3 + 0.5, 1e-12);
}

TEST(GaussianPointMath, EvalSHClampsNegativeRadianceToZero)
{
	const glm::dvec3 dc(-10.0, -10.0, -10.0); // C0*dc + 0.5 << 0
	const glm::dvec3 got = evalSH(0, dc, nullptr, glm::dvec3(1, 0, 0));
	EXPECT_DOUBLE_EQ(got.r, 0.0);
	EXPECT_DOUBLE_EQ(got.g, 0.0);
	EXPECT_DOUBLE_EQ(got.b, 0.0);
}

TEST(GaussianPointMath, EvalSHBand1IsViewDependent)
{
	// Degree 1: 3 rest coeffs per channel, channel-major.
	// Put a signal only on the first band-1 coefficient of the red channel.
	std::vector<double> rest(9, 0.0);
	rest[0] = 1.0; // R, coeff 0  -> basis -C1 * y
	const glm::dvec3 dc(0.0);

	constexpr double C1 = 0.4886025119029199;
	const glm::dvec3 up = evalSH(1, dc, rest.data(), glm::dvec3(0, 1, 0));
	const glm::dvec3 dn = evalSH(1, dc, rest.data(), glm::dvec3(0, -1, 0));

	// up:  -C1 * (+1) * 1  + 0.5 ; dn: -C1 * (-1) * 1 + 0.5
	EXPECT_NEAR(up.r, std::max(-C1 + 0.5, 0.0), 1e-12);
	EXPECT_NEAR(dn.r, C1 + 0.5, 1e-12);
	EXPECT_NEAR(up.g, 0.5, 1e-12);
}

TEST(GaussianPointMath, EvalSHHigherDegreesStayFinite)
{
	std::vector<double> rest(45, 0.3);
	const glm::dvec3 got = evalSH(3, glm::dvec3(0.1), rest.data(),
	                              glm::normalize(glm::dvec3(0.3, -0.7, 0.5)));
	EXPECT_TRUE(std::isfinite(got.r) && std::isfinite(got.g) && std::isfinite(got.b));
	EXPECT_GE(got.r, 0.0);
}

TEST(GaussianPointMath, EvalSHLowerDegreeUsesLoadedDataStride)
{
    // Degree-3 storage has 15 coefficients per channel. Evaluating only degree 1
    // must still address G/B at offsets 15/30, not at offsets 3/6.
    std::array<double, 45> rest{};
    rest[0] = 1.0;
    rest[15] = 2.0;
    rest[30] = 3.0;
    const glm::dvec3 dir(0.0, -1.0, 0.0);
    const glm::dvec3 got = evalSH(1, glm::dvec3(0.0), rest.data(), dir, 15);
    EXPECT_NEAR(got.r, 0.5 + 0.4886025119029199 * 1.0, 1e-12);
    EXPECT_NEAR(got.g, 0.5 + 0.4886025119029199 * 2.0, 1e-12);
    EXPECT_NEAR(got.b, 0.5 + 0.4886025119029199 * 3.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Depth / colour packing
// ---------------------------------------------------------------------------

TEST(GaussianPointMath, OrderedDepthKeyMatchesTestVectors)
{
	for (const auto& c : vec::kDepthKey)
		EXPECT_EQ(orderedDepthKey(c.depth), c.key) << "depth=" << c.depth;
}

TEST(GaussianPointMath, OrderedDepthKeyIsMonotoneAndInvertible)
{
	const float ds[] = { -3.0f, -0.5f, 0.0f, 1e-6f, 0.25f, 1.0f, 7.5f, 1000.0f };
	for (size_t i = 1; i < std::size(ds); ++i)
		EXPECT_LT(orderedDepthKey(ds[i - 1]), orderedDepthKey(ds[i]))
			<< ds[i - 1] << " vs " << ds[i];
	for (float d : ds)
		EXPECT_FLOAT_EQ(orderedDepthKeyInverse(orderedDepthKey(d)), d);
}

TEST(GaussianPointMath, PackDepthColorRoundTrips)
{
	const std::uint32_t rgba = 0x11AA33FFu;
	const std::uint64_t packed = packDepthColor(2.5f, rgba);
	EXPECT_EQ(unpackColor(packed), rgba);
	EXPECT_EQ(unpackDepthKey(packed), orderedDepthKey(2.5f));

	// atomicMin on the 64-bit key keeps the nearer sample regardless of colour.
	const std::uint64_t nearer = packDepthColor(1.0f, 0x00000000u);
	const std::uint64_t farther = packDepthColor(9.0f, 0xFFFFFFFFu);
	EXPECT_LT(nearer, farther);
}
