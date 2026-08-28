#include "pch.h"
#include "../PointCloud/RansacConeDetector.h"

#include <random>
#include <vector>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	constexpr auto kPi = 3.14159265358979323846;

	std::vector<Vector3df> createNoisyConePoints(
		size_t inliers,
		size_t outliers,
		const Vector3df& apex = Vector3df(0.0f, 0.0f, 0.0f),
		const Vector3df& axis = Vector3df(0.0f, 0.0f, 1.0f),
		float halfAngleRad = 0.20943951f, // 12 degrees
		float sMin = 1.0f,
		float sMax = 5.0f,
		float radialNoiseSigma = 0.01f,
		unsigned seed = 12345)
	{
		std::mt19937 rng(seed);
		std::uniform_real_distribution<float> angDist(0.0f, 2.0f * static_cast<float>(kPi));
		std::uniform_real_distribution<float> sDist(sMin, sMax);
		std::normal_distribution<float> radialNoise(0.0f, radialNoiseSigma);
		std::uniform_real_distribution<float> outlierPos(-5.0f, 5.0f);

		// Build an orthonormal basis (u, v) perpendicular to axis for the azimuthal sweep.
		Vector3df helper(1.0f, 0.0f, 0.0f);
		if (std::fabs(axis.x) > 0.9f) helper = Vector3df(0.0f, 1.0f, 0.0f);
		const Vector3df u = glm::normalize(glm::cross(axis, helper));
		const Vector3df v = glm::cross(axis, u);
		const float tanHalf = std::tan(halfAngleRad);

		std::vector<Vector3df> pts;
		pts.reserve(inliers + outliers);

		for (size_t i = 0; i < inliers; ++i) {
			const float s = sDist(rng);
			const float a = angDist(rng);
			const float r = s * tanHalf + radialNoise(rng);
			pts.push_back(apex + axis * s + u * (r * std::cos(a)) + v * (r * std::sin(a)));
		}

		for (size_t i = 0; i < outliers; ++i) {
			pts.emplace_back(outlierPos(rng), outlierPos(rng), outlierPos(rng));
		}

		return pts;
	}

} // namespace

TEST(RansacConeDetectorTest, DetectsConeWithNoise)
{
	const size_t inliers = 500;
	const size_t outliers = 100;
	const Vector3df trueApex(0.0f, 0.0f, 0.0f);
	const Vector3df trueAxis(0.0f, 0.0f, 1.0f);
	const float trueHalfAngle = 0.20943951f; // 12 degrees
	auto points = createNoisyConePoints(inliers, outliers, trueApex, trueAxis, trueHalfAngle, 1.0f, 5.0f, 0.01f, 12345u);

	RansacConeDetector detector(42u);
	RansacConeDetector::ConeModel model;
	const bool ok = detector.detect(points, model, 300, 0.05f, 100);

	EXPECT_TRUE(ok);
	EXPECT_GE(model.inliers.size(), size_t(100));
	EXPECT_NEAR(model.halfAngleRad, trueHalfAngle, 0.05f);

	const float axisAlignment = glm::dot(model.axis, trueAxis);
	EXPECT_GT(axisAlignment, 0.9f);

	// The apex should lie reasonably close to the true apex. A wider tolerance than the angle/axis
	// checks: for a shallow cone, a tiny error in the fitted slope corresponds to a much larger
	// error in the radius==0 crossing point (apex position), so this is inherently noisier.
	EXPECT_NEAR(glm::length(model.apex - trueApex), 0.0f, 1.0f);
}

TEST(RansacConeDetectorTest, FailsWhenNotEnoughInliers)
{
	const size_t inliers = 40;
	const size_t outliers = 20;
	auto points = createNoisyConePoints(inliers, outliers);

	RansacConeDetector detector(42u);
	RansacConeDetector::ConeModel model;
	const bool ok = detector.detect(points, model, 200, 0.05f, 100);

	EXPECT_FALSE(ok);
}

TEST(RansacConeDetectorTest, FailsWithLessThanSixPoints)
{
	std::vector<Vector3df> points = {
		Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 1.0f),
		Vector3df(0.0f, 1.0f, 1.0f), Vector3df(-1.0f, 0.0f, 1.0f)
	};
	RansacConeDetector detector(42u);
	RansacConeDetector::ConeModel model;
	EXPECT_FALSE(detector.detect(points, model));
}
