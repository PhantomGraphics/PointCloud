#include "pch.h"
#include "../PointCloud/RansacCylinderDetector.h"

#include <random>
#include <vector>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	constexpr auto kPi = 3.14159265358979323846;

    static std::vector<Vector3df> createNoisyCylinderPoints(
        size_t inliers,
        size_t outliers,
        float radius = 1.0f,
        float height = 2.0f,
        float radialNoiseSigma = 0.01f,
        float axialNoiseSigma = 0.005f,
        unsigned seed = 12345)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> angDist(0.0f, 2.0f * static_cast<float>(kPi));
        std::uniform_real_distribution<float> hDist(-height * 0.5f, height * 0.5f);
        std::normal_distribution<float> radialNoise(0.0f, radialNoiseSigma);
        std::normal_distribution<float> axialNoise(0.0f, axialNoiseSigma);
        std::uniform_real_distribution<float> outlierPos(-2.0f, 2.0f);

        std::vector<Vector3df> pts;
        pts.reserve(inliers + outliers);

        // Cylinder along the Z axis.
        for (size_t i = 0; i < inliers; ++i) {
            const float a = angDist(rng);
            const float hval = hDist(rng);
            const float r = radius + radialNoise(rng);
            const float x = std::cos(a) * r;
            const float y = std::sin(a) * r;
            const float z = hval + axialNoise(rng);
            pts.emplace_back(x, y, z);
        }

        // Random outliers.
        for (size_t i = 0; i < outliers; ++i) {
            const float x = outlierPos(rng);
            const float y = outlierPos(rng);
            const float z = outlierPos(rng);
            pts.emplace_back(x, y, z);
        }

        return pts;
    }
}

TEST(RansacCylinderDetectorTest, DetectsCylinderWithNoise)
{
    const size_t inliers = 500;
    const size_t outliers = 100;
    const float trueRadius = 1.0f;
    auto points = createNoisyCylinderPoints(inliers, outliers, trueRadius, 2.0f, 0.01f, 0.005f, 12345u);

    RansacCylinderDetector detector;
    RansacCylinderDetector::CylinderModel model;
    const bool ok = detector.detect(points, model, 200, 0.03f, 50);

    EXPECT_TRUE(ok);
    EXPECT_GE(model.inliers.size(), size_t(50));
    EXPECT_NEAR(model.radius, trueRadius, 0.05f);
    const float zAlignment = std::abs(model.direction.z);
    EXPECT_GE(zAlignment, 0.9f);
}

TEST(RansacCylinderDetectorTest, FailsWhenNotEnoughInliers)
{
    const size_t inliers = 40;
    const size_t outliers = 20;
    auto points = createNoisyCylinderPoints(inliers, outliers, 1.0f, 2.0f, 0.01f, 0.005f, 4242u);

    RansacCylinderDetector detector;
    RansacCylinderDetector::CylinderModel model;
    const int iterations = 200;
    const float distThreshold = 0.03f;
    const size_t minInliers = 100;

    const bool ok = detector.detect(points, model, iterations, distThreshold, minInliers);
    EXPECT_FALSE(ok);
}

TEST(RansacCylinderDetectorTest, FailsWithLessThanThreePoints)
{
    std::vector<Vector3df> points = { Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f) };
    RansacCylinderDetector detector;
    RansacCylinderDetector::CylinderModel model;
    EXPECT_FALSE(detector.detect(points, model));
}
