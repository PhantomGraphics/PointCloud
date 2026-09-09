#include "GaussianPointMath.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace GSView::gpm {

namespace {
inline constexpr double kPi    = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
} // namespace

// ===========================================================================
// Scalar activations
// ===========================================================================

double sigmoid(double x)
{
	return 1.0 / (1.0 + std::exp(-x));
}

double shDcToColor(double sh)
{
	return std::clamp(sh * kShC0 + 0.5, 0.0, 1.0);
}

glm::dvec3 evalSH(int degree, const glm::dvec3& dc, const double* rest, const glm::dvec3& dir)
{
	// Coefficients from 3DGS sh_utils.eval_sh.
	constexpr double C1 = 0.4886025119029199;
	constexpr double C2[5] = { 1.0925484305920792, -1.0925484305920792, 0.31539156525252005,
	                           -1.0925484305920792, 0.5462742152960396 };
	constexpr double C3[7] = { -0.5900435899266435, 2.890611442640554, -0.4570457994644658,
	                           0.3731763325901154, -0.4570457994644658, 1.445305721320277,
	                           -0.5900435899266435 };

	glm::dvec3 result = kShC0 * dc;
	degree = std::clamp(degree, 0, 3);

	if (degree >= 1 && rest) {
		const int R = shRestPerChannel(degree);
		auto sh = [&](int k) {
			return glm::dvec3(rest[0 * R + k], rest[1 * R + k], rest[2 * R + k]);
		};
		const double x = dir.x, y = dir.y, z = dir.z;

		result += -C1 * y * sh(0) + C1 * z * sh(1) - C1 * x * sh(2);

		if (degree >= 2) {
			const double xx = x*x, yy = y*y, zz = z*z;
			const double xy = x*y, yz = y*z, xz = x*z;
			result += C2[0]*xy*sh(3) + C2[1]*yz*sh(4) + C2[2]*(2.0*zz - xx - yy)*sh(5)
			        + C2[3]*xz*sh(6) + C2[4]*(xx - yy)*sh(7);

			if (degree >= 3) {
				result += C3[0]*y*(3.0*xx - yy)*sh(8)
				        + C3[1]*xy*z*sh(9)
				        + C3[2]*y*(4.0*zz - xx - yy)*sh(10)
				        + C3[3]*z*(2.0*zz - 3.0*xx - 3.0*yy)*sh(11)
				        + C3[4]*x*(4.0*zz - xx - yy)*sh(12)
				        + C3[5]*z*(xx - yy)*sh(13)
				        + C3[6]*x*(xx - 3.0*yy)*sh(14);
			}
		}
	}

	result += 0.5;
	return glm::max(result, glm::dvec3(0.0));
}

// ===========================================================================
// Covariance
// ===========================================================================

glm::dmat3 quatToRotation(const glm::dquat& rot)
{
	return glm::mat3_cast(glm::normalize(rot));
}

glm::dmat3 covariance3D(const glm::dvec3& logScale, const glm::dquat& rot)
{
	const glm::dmat3 R = quatToRotation(rot);
	glm::dmat3 S2(0.0);
	S2[0][0] = std::exp(2.0 * logScale.x);
	S2[1][1] = std::exp(2.0 * logScale.y);
	S2[2][2] = std::exp(2.0 * logScale.z);
	return R * S2 * glm::transpose(R);
}

glm::dvec3 worldToCamera(const PinholeCamera& cam, const glm::dvec3& world)
{
	return cam.viewRot * (world - cam.viewPos);
}

glm::dmat2 covariance2D(const glm::dmat3& cov3d,
                        const glm::dvec3& meanWorld,
                        const PinholeCamera& cam,
                        double lowPass)
{
	glm::dvec3 t = worldToCamera(cam, meanWorld);

	// Frustum clamp (3DGS: keeps the Jacobian well-conditioned near screen edges).
	const double limx = 1.3 * cam.tanFovX;
	const double limy = 1.3 * cam.tanFovY;
	const double txtz = t.x / t.z;
	const double tytz = t.y / t.z;
	t.x = std::clamp(txtz, -limx, limx) * t.z;
	t.y = std::clamp(tytz, -limy, limy) * t.z;

	// Mathematical perspective Jacobian (rows: du,dv; cols: dx,dy,dz), padded to
	// 3x3. glm is column-major, so element (row r, col c) is J[c][r].
	glm::dmat3 J(0.0);
	J[0][0] = cam.focalX / t.z;
	J[2][0] = -cam.focalX * t.x / (t.z * t.z);
	J[1][1] = cam.focalY / t.z;
	J[2][1] = -cam.focalY * t.y / (t.z * t.z);

	const glm::dmat3 T = J * cam.viewRot;
	const glm::dmat3 cov = T * cov3d * glm::transpose(T);

	glm::dmat2 c2(cov[0][0], cov[0][1], cov[1][0], cov[1][1]);
	c2[0][0] += lowPass;
	c2[1][1] += lowPass;
	return c2;
}

glm::dmat2 cholesky2D(const glm::dmat2& A)
{
	const double eps = 1e-300;
	const double l00 = std::sqrt(std::max(A[0][0], eps));
	const double l10 = A[0][1] / l00;
	const double l11 = std::sqrt(std::max(A[1][1] - l10 * l10, eps));
	glm::dmat2 L(0.0);
	L[0][0] = l00;
	L[0][1] = l10;
	L[1][1] = l11;
	return L;
}

// ===========================================================================
// Dilogarithm
// ===========================================================================

namespace {

// sum_{k>=1} x^k / k^2, converges fast for |x| <= 0.5.
double dilogSeries(double x)
{
	double term = x;
	double sum = x;
	for (int k = 2; k < 1000; ++k) {
		term *= x;
		const double add = term / (static_cast<double>(k) * k);
		sum += add;
		if (std::abs(add) < 1e-18 * std::abs(sum)) break;
	}
	return sum;
}

} // namespace

double dilog(double x)
{
	if (x >= 1.0)  return kDilogAtOne;
	if (x <= -1.0) {
		if (x == -1.0) return -kDilogAtOne / 2.0;
		// Outside [-1,1] is not needed by this module; clamp defensively.
		x = -1.0;
		return -kDilogAtOne / 2.0;
	}
	if (x == 0.0) return 0.0;

	if (x < 0.0) {
		// Landen: Li2(x) = -Li2(x/(x-1)) - 1/2 ln^2(1-x),  x/(x-1) in [0, 0.5].
		const double y = x / (x - 1.0);
		const double l = std::log(1.0 - x);
		return -dilogSeries(y) - 0.5 * l * l;
	}
	if (x <= 0.5) return dilogSeries(x);

	// Reflection: Li2(x) = pi^2/6 - ln(x) ln(1-x) - Li2(1-x).
	return kDilogAtOne - std::log(x) * std::log(1.0 - x) - dilogSeries(1.0 - x);
}

double invDilog(double target)
{
	target = std::clamp(target, 0.0, std::nextafter(kDilogAtOne, 0.0));
	if (target <= 0.0) return 0.0;

	// dilog is smooth and strictly increasing on [0,1). Safeguarded Newton: a
	// Newton step (f' = -ln(1-y)/y) when it stays inside the current bracket,
	// bisection otherwise. Converges in well under 10 iterations.
	double lo = 0.0, hi = 1.0;
	double y = target / kDilogAtOne;   // in (0,1); good seed near both ends
	for (int i = 0; i < 40; ++i) {
		const double f = dilog(y) - target;
		if (f > 0.0) hi = y; else lo = y;

		const double dfdy = (y > 1e-12 && y < 1.0 - 1e-15)
		                        ? (-std::log(1.0 - y) / y) : 1.0;
		double ny = y - f / dfdy;
		if (!(ny > lo && ny < hi)) ny = 0.5 * (lo + hi);
		if (std::abs(ny - y) <= 1e-14 * (1.0 + y)) return ny;
		y = ny;
	}
	return y;
}

// ===========================================================================
// Expected point count
// ===========================================================================

double expectedPointCount(double detCov2d, double opacity)
{
	if (detCov2d <= 0.0 || opacity <= 0.0) return 0.0;
	const double o = std::min(opacity, 1.0);
	return 2.0 * kPi * std::sqrt(detCov2d) * dilog(o);
}

// ===========================================================================
// Sampling
// ===========================================================================

std::uint32_t pcgHash(std::uint32_t v)
{
	std::uint32_t s = v * 747796405u + 2891336453u;
	std::uint32_t w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
	return (w >> 22u) ^ w;
}

std::uint32_t particleSeed(SeedMode mode,
                           std::uint32_t splatId,
                           std::uint32_t particleIndex,
                           std::uint32_t frameIndex)
{
	std::uint32_t s = pcgHash(splatId * 747796405u + particleIndex + 1u);
	if (mode == SeedMode::FrameVarying)
		s = pcgHash(s ^ (frameIndex * 2654435761u + 1u));
	return s ? s : 1u;
}

double RandStream::next()
{
	state_ = pcgHash(state_);
	return static_cast<double>(state_) * (1.0 / 4294967296.0);
}

double RandStream::normal()
{
	if (haveSpare_) {
		haveSpare_ = false;
		return spare_;
	}
	const double u1 = std::max(next(), 1e-300);
	const double u2 = next();
	const double r = std::sqrt(-2.0 * std::log(u1));
	const double th = kTwoPi * u2;
	spare_ = r * std::sin(th);
	haveSpare_ = true;
	return r * std::cos(th);
}

int poissonSample(double lambda, RandStream& rng)
{
	if (lambda <= 0.0) return 0;
	if (lambda < 30.0) {
		const double L = std::exp(-lambda);
		int k = 0;
		double p = 1.0;
		do {
			++k;
			p *= rng.next();
		} while (p > L);
		return k - 1;
	}
	const long n = std::lround(lambda + std::sqrt(lambda) * rng.normal());
	return static_cast<int>(std::max(0L, n));
}

double sampleCorrectedRadius(double opacity, double u)
{
	const double o = std::clamp(opacity, 0.0, 1.0);
	if (o <= 0.0) return 0.0;
	u = std::clamp(u, 0.0, 1.0);

	const double D = dilog(o);
	const double target = (1.0 - u) * D;      // want Li2(o * e^{-r^2/2}) == target
	double y = std::min(invDilog(target), o);
	if (y <= 0.0) return 40.0;                 // effective cutoff radius
	const double r2 = -2.0 * std::log(y / o);
	return std::sqrt(std::max(0.0, r2));
}

glm::dvec2 sampleCorrectedOffset(double opacity, const glm::dmat2& cov2d, RandStream& rng)
{
	const double r = sampleCorrectedRadius(opacity, rng.next());
	const double theta = kTwoPi * rng.next();
	const glm::dvec2 whitened(r * std::cos(theta), r * std::sin(theta));
	return cholesky2D(cov2d) * whitened;
}

// ===========================================================================
// Analytic per-pixel coverage oracle
// ===========================================================================

double alphaAt(const Gaussian2D& g, const glm::dvec2& pixel)
{
	const glm::dvec2 d = pixel - g.mean;
	const glm::dmat2 inv = glm::inverse(g.cov);
	const double m = glm::dot(d, inv * d);
	return std::clamp(g.opacity, 0.0, 1.0) * std::exp(-0.5 * m);
}

double expectedCoverage(double alpha, double pixelArea)
{
	alpha = std::clamp(alpha, 0.0, 1.0 - 1e-15);
	return 1.0 - std::pow(1.0 - alpha, pixelArea);
}

glm::dvec4 over(const glm::dvec4& src, const glm::dvec4& dst, AlphaMode mode)
{
	const double sa = std::clamp(src.a, 0.0, 1.0);
	const double da = std::clamp(dst.a, 0.0, 1.0);
	const double outA = sa + da * (1.0 - sa);

	if (mode == AlphaMode::Premultiplied) {
		return glm::dvec4(glm::dvec3(src) + glm::dvec3(dst) * (1.0 - sa), outA);
	}
	// Straight
	if (outA <= 0.0) return glm::dvec4(0.0);
	const glm::dvec3 rgb =
		(glm::dvec3(src) * sa + glm::dvec3(dst) * da * (1.0 - sa)) / outA;
	return glm::dvec4(rgb, outA);
}

glm::dvec3 compositeExpected(std::vector<Gaussian2D> layers,
                             const glm::dvec2& pixel,
                             const glm::dvec3& background,
                             double pixelArea)
{
	std::sort(layers.begin(), layers.end(),
	          [](const Gaussian2D& a, const Gaussian2D& b) { return a.depth < b.depth; });

	glm::dvec3 accum(0.0);   // premultiplied colour built up front-to-back
	double transmittance = 1.0;
	for (const Gaussian2D& g : layers) {
		const double a = expectedCoverage(alphaAt(g, pixel), pixelArea);
		accum += transmittance * a * g.color;
		transmittance *= (1.0 - a);
		if (transmittance <= 1e-9) break;
	}
	return accum + transmittance * background;
}

// ===========================================================================
// Depth / colour packing
// ===========================================================================

std::uint32_t orderedDepthKey(float depth)
{
	std::uint32_t bits;
	std::memcpy(&bits, &depth, sizeof(bits));
	const std::uint32_t mask = (bits & 0x80000000u) ? 0xFFFFFFFFu : 0x80000000u;
	return bits ^ mask;
}

float orderedDepthKeyInverse(std::uint32_t key)
{
	const std::uint32_t mask = (key & 0x80000000u) ? 0x80000000u : 0xFFFFFFFFu;
	const std::uint32_t bits = key ^ mask;
	float depth;
	std::memcpy(&depth, &bits, sizeof(depth));
	return depth;
}

std::uint64_t packDepthColor(float depth, std::uint32_t rgba8)
{
	return (static_cast<std::uint64_t>(orderedDepthKey(depth)) << 32) | rgba8;
}

std::uint32_t unpackColor(std::uint64_t packed)
{
	return static_cast<std::uint32_t>(packed & 0xFFFFFFFFu);
}

std::uint32_t unpackDepthKey(std::uint64_t packed)
{
	return static_cast<std::uint32_t>(packed >> 32);
}

} // namespace GSView::gpm
