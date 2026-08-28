#include "pch.h"
#include "../PointCloud/RansacPlaneDetector.h"

#include <random>
#include <vector>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

static std::vector<Vector3df> createNoisyPlanePoints(
    size_t inliers,
    size_t outliers,
    float noiseSigma,
    unsigned seed = 12345)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uni(-1.0f, 1.0f);
    std::uniform_real_distribution<float> outZ(0.5f, 1.0f);
    std::normal_distribution<float> gauss(0.0f, noiseSigma);

    std::vector<Vector3df> pts;
    pts.reserve(inliers + outliers);

    // 平面: z = 0 (法線 (0,0,1), offset = 0)
    for (size_t i = 0; i < inliers; ++i) {
        float x = uni(rng);
        float y = uni(rng);
        float z = gauss(rng);
        pts.emplace_back(x, y, z);
    }

    // 外れ値
    for (size_t i = 0; i < outliers; ++i) {
        float x = uni(rng);
        float y = uni(rng);
        float z = outZ(rng);
        pts.emplace_back(x, y, z);
    }

    return pts;
}

TEST(RansacPlaneDetectorTest, DetectsPlaneWithNoise)
{
    const size_t inliers = 300;
    const size_t outliers = 50;
    const float noiseSigma = 0.005f;
    auto points = createNoisyPlanePoints(inliers, outliers, noiseSigma, 9876u);

   RansacPlaneDetector detector(42u);
    RansacPlaneDetector::PlaneModel model;
    const int iterations = 200;
    const float distThreshold = 0.02f;
    const size_t minInliers = 100;

    const bool ok = detector.detect(points, model, iterations, distThreshold, minInliers);

    EXPECT_TRUE(ok);
    EXPECT_GE(model.inliers.size(), minInliers);

    // 平面は z=0 に近いはず（法線は (0,0,±1) に近い）
    const float absDotZ = std::fabs(model.normal.z);
    EXPECT_GT(absDotZ, 0.98f);

    // オフセットは 0 に近い
    EXPECT_NEAR(model.offset, 0.0f, 0.02f);
}

TEST(RansacPlaneDetectorTest, FailsWhenNotEnoughInliers)
{
    const size_t inliers = 30; // 少なすぎる
    const size_t outliers = 20;
    const float noiseSigma = 0.005f;
    auto points = createNoisyPlanePoints(inliers, outliers, noiseSigma, 1111u);

   RansacPlaneDetector detector(42u);
    RansacPlaneDetector::PlaneModel model;
    const int iterations = 200;
    const float distThreshold = 0.02f;
    const size_t minInliers = 50;

    const bool ok = detector.detect(points, model, iterations, distThreshold, minInliers);

    EXPECT_FALSE(ok);
}

TEST(RansacPlaneDetectorTest, FailsWithLessThanThreePoints)
{
    std::vector<Vector3df> points = { Vector3df(0.0f, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f) };
   RansacPlaneDetector detector(42u);
    RansacPlaneDetector::PlaneModel model;
    EXPECT_FALSE(detector.detect(points, model));
}