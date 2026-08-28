#include "pch.h"
#include "../PointCloud/RansacSphereDetector.h"

#include <random>
#include <vector>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	constexpr auto kPi = 3.14159265358979323846;

	std::vector<Vector3df> createNoisySpherePoints(
		size_t inliers,
		size_t outliers,
		const Vector3df& center = Vector3df(0.0f, 0.0f, 0.0f),
		float radius = 1.0f,
		float radialNoiseSigma = 0.005f,
		unsigned seed = 12345)
	{
		std::mt19937 rng(seed);
		std::uniform_real_distribution<float> thetaDist(0.0f, static_cast<float>(kPi));
		std::uniform_real_distribution<float> phiDist(0.0f, 2.0f * static_cast<float>(kPi));
		std::normal_distribution<float> radialNoise(0.0f, radialNoiseSigma);
		std::uniform_real_distribution<float> outlierPos(-3.0f, 3.0f);

		std::vector<Vector3df> pts;
		pts.reserve(inliers + outliers);

		for (size_t i = 0; i < inliers; ++i) {
			const float theta = thetaDist(rng);
			const float phi = phiDist(rng);
			const float r = radius + radialNoise(rng);
			pts.push_back(center + Vector3df(
				r * std::sin(theta) * std::cos(phi),
				r * std::sin(theta) * std::sin(phi),
				r * std::cos(theta)));
		}

		for (size_t i = 0; i < outliers; ++i) {
			pts.emplace_back(outlierPos(rng), outlierPos(rng), outlierPos(rng));
		}

		return pts;
	}

} // namespace

TEST(RansacSphereDetectorTest, DetectsSphereWithNoise)
{
	const size_t inliers = 400;
	const size_t outliers = 80;
	const Vector3df trueCenter(0.3f, -0.2f, 0.1f);
	const float trueRadius = 1.5f;
	auto points = createNoisySpherePoints(inliers, outliers, trueCenter, trueRadius, 0.005f, 12345u);

	RansacSphereDetector detector(42u);
	RansacSphereDetector::SphereModel model;
	const bool ok = detector.detect(points, model, 300, 0.03f, 100);

	EXPECT_TRUE(ok);
	EXPECT_GE(model.inliers.size(), size_t(100));
	EXPECT_NEAR(model.radius, trueRadius, 0.05f);
	EXPECT_NEAR(model.center.x, trueCenter.x, 0.05f);
	EXPECT_NEAR(model.center.y, trueCenter.y, 0.05f);
	EXPECT_NEAR(model.center.z, trueCenter.z, 0.05f);
}

TEST(RansacSphereDetectorTest, FailsWhenNotEnoughInliers)
{
	const size_t inliers = 30;
	const size_t outliers = 20;
	auto points = createNoisySpherePoints(inliers, outliers, Vector3df(0, 0, 0), 1.0f, 0.005f, 4242u);

	RansacSphereDetector detector(42u);
	RansacSphereDetector::SphereModel model;
	const bool ok = detector.detect(points, model, 200, 0.03f, 100);

	EXPECT_FALSE(ok);
}

TEST(RansacSphereDetectorTest, FailsWithLessThanFourPoints)
{
	std::vector<Vector3df> points = {
		Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f)
	};
	RansacSphereDetector detector(42u);
	RansacSphereDetector::SphereModel model;
	EXPECT_FALSE(detector.detect(points, model));
}

TEST(RansacSphereDetectorTest, FailsWithCoplanarPoints)
{
	// All points lie on z=0: no unique sphere can be fit (degenerate 4-point systems).
	std::vector<Vector3df> points = {
		Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f),
		Vector3df(0.0f, 1.0f, 0.0f), Vector3df(1.0f, 1.0f, 0.0f),
		Vector3df(0.5f, 0.5f, 0.0f), Vector3df(2.0f, 0.0f, 0.0f)
	};
	RansacSphereDetector detector(42u);
	RansacSphereDetector::SphereModel model;
	EXPECT_FALSE(detector.detect(points, model, 200, 0.02f, 4));
}
