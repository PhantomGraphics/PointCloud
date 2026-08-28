#include "pch.h"
#include "../PointCloud/ICPRegistration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

std::vector<Vector3df> makeSpherePoints(size_t count, float radius, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uniTheta(0.0f, 3.14159265358979323846f);
    std::uniform_real_distribution<float> uniPhi(0.0f, 2.0f * 3.14159265358979323846f);

    std::vector<Vector3df> pts;
    pts.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const float theta = uniTheta(rng);
        const float phi = uniPhi(rng);
        pts.emplace_back(
            radius * std::sin(theta) * std::cos(phi),
            radius * std::sin(theta) * std::sin(phi),
            radius * std::cos(theta));
    }
    return pts;
}

std::vector<Vector3df> translate(const std::vector<Vector3df>& pts, const Vector3df& offset)
{
    std::vector<Vector3df> result;
    result.reserve(pts.size());
    for (const auto& p : pts) result.push_back(p + offset);
    return result;
}

std::vector<Vector3df> rotateZ(const std::vector<Vector3df>& pts, float angleRad)
{
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);
    std::vector<Vector3df> result;
    result.reserve(pts.size());
    for (const auto& p : pts) {
        result.emplace_back(c * p.x - s * p.y, s * p.x + c * p.y, p.z);
    }
    return result;
}

// A rod-shaped random point cloud, elongated along X -- unlike a sphere, it has no rotational
// symmetry about Z, so ICP has a single, unambiguous optimum to converge to.
std::vector<Vector3df> makeRodPoints(size_t count, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uniX(-2.0f, 2.0f);
    std::uniform_real_distribution<float> uniYZ(-0.3f, 0.3f);

    std::vector<Vector3df> pts;
    pts.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        pts.emplace_back(uniX(rng), uniYZ(rng), uniYZ(rng));
    }
    return pts;
}

float nearestDistance(const Vector3df& p, const std::vector<Vector3df>& cloud)
{
    float best = std::numeric_limits<float>::max();
    for (const auto& q : cloud) {
        best = std::min(best, glm::length(q - p));
    }
    return best;
}

} // namespace

TEST(ICPRegistrationTest, FailsWithEmptyInput)
{
    ICPRegistration icp;
    ICPRegistration::Result result;
    EXPECT_FALSE(icp.align({}, { Vector3df(0, 0, 0) }, result));
    EXPECT_FALSE(icp.align({ Vector3df(0, 0, 0) }, {}, result));
}

TEST(ICPRegistrationTest, FailsWithNonPositiveMaxIterations)
{
    ICPRegistration icp;
    ICPRegistration::Result result;
    auto pts = makeSpherePoints(50, 1.0f, 1);
    EXPECT_FALSE(icp.align(pts, pts, result, 0));
}

TEST(ICPRegistrationTest, RecoversExactTranslation)
{
    // Offset kept well below the ~0.2 mean nearest-neighbor spacing of 300 points on a unit
    // sphere, so nearest-neighbor correspondence is unambiguous from the first iteration.
    const auto target = makeSpherePoints(300, 1.0f, 42);
    const Vector3df offset(0.05f, -0.03f, 0.02f);
    const auto source = translate(target, offset);

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(source, target, result, 50, 1.0e-8f));

    EXPECT_NEAR(result.fitness, 0.0f, 1.0e-4f);

    // Applying the recovered transform to `source` should land back on `target`.
    for (size_t i = 0; i < source.size(); ++i) {
        const auto aligned = ICPRegistration::transformPoint(result, source[i]);
        EXPECT_NEAR(aligned.x, target[i].x, 1.0e-2f);
        EXPECT_NEAR(aligned.y, target[i].y, 1.0e-2f);
        EXPECT_NEAR(aligned.z, target[i].z, 1.0e-2f);
    }
}

TEST(ICPRegistrationTest, RecoversRotationAndTranslation)
{
    // A sphere would have a rotational symmetry ambiguity around its center; use an elongated
    // rod-shaped cloud instead so the correct alignment is unambiguous.
    const auto target = makeRodPoints(400, 7);
    auto rotated = rotateZ(target, 0.08f);
    const Vector3df offset(0.1f, 0.05f, -0.05f);
    const auto source = translate(rotated, offset);

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(source, target, result, 100, 1.0e-8f));

    // Every transformed source point should land very close to *some* target point (checked via
    // nearest-neighbor distance rather than matching indices, since ICP itself only guarantees
    // shape alignment, not recovering the original point correspondence).
    for (size_t i = 0; i < source.size(); i += 23) {
        const auto aligned = ICPRegistration::transformPoint(result, source[i]);
        EXPECT_LT(nearestDistance(aligned, target), 0.05f);
    }
}

TEST(ICPRegistrationTest, RejectsCorrespondencesBeyondMaxDistance)
{
    // Two clusters far enough apart that with a tight maxCorrespondenceDistance,
    // no correspondence survives on the first iteration.
    std::vector<Vector3df> target = { Vector3df(0, 0, 0), Vector3df(1, 0, 0), Vector3df(0, 1, 0), Vector3df(0, 0, 1) };
    std::vector<Vector3df> source = translate(target, Vector3df(100.0f, 100.0f, 100.0f));

    ICPRegistration icp;
    ICPRegistration::Result result;
    EXPECT_FALSE(icp.align(source, target, result, 10, 1.0e-6f, 0.5f));
}

TEST(ICPRegistrationTest, IdenticalCloudsConvergeImmediatelyWithIdentity)
{
    const auto pts = makeSpherePoints(100, 1.0f, 99);

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(pts, pts, result, 20, 1.0e-8f));

    EXPECT_NEAR(result.fitness, 0.0f, 1.0e-6f);
    EXPECT_TRUE(areSame(result.rotation, identitiyMatrix3d<float>(), 1.0e-3f));
    EXPECT_NEAR(glm::length(result.translation), 0.0f, 1.0e-3f);
    EXPECT_FLOAT_EQ(result.scale, 1.0f); // estimateScale defaults to false: always rigid.
}

TEST(ICPRegistrationTest, EstimateScaleRecoversUniformScale)
{
    // source is target scaled up by 1.5x around the origin (both spheres share a center,
    // so no translation/rotation is needed -- isolates the scale estimate).
    const auto target = makeSpherePoints(300, 1.0f, 5);
    const float trueScale = 1.5f;
    std::vector<Vector3df> source;
    source.reserve(target.size());
    for (const auto& p : target) source.push_back(p * trueScale);

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(source, target, result, 50, 1.0e-8f, 0.0f,
        ICPRegistration::RobustKernel::None, 1.0f, /*estimateScale=*/true));

    // align() fits source -> target, so the recovered scale is the inverse of trueScale.
    EXPECT_NEAR(result.scale, 1.0f / trueScale, 0.05f);

    for (size_t i = 0; i < source.size(); i += 23) {
        const auto aligned = ICPRegistration::transformPoint(result, source[i]);
        EXPECT_LT(nearestDistance(aligned, target), 0.05f);
    }
}

TEST(ICPRegistrationTest, RobustKernelDownweightsOutlierCorrespondences)
{
    // Base cloud is already perfectly aligned (source == target for the inliers), so the
    // ideal fit is the identity transform.
    auto target = makeSpherePoints(200, 1.0f, 11);
    auto source = target;

    // A handful of source points far outside the sphere have no good match in target
    // (point-to-point residual ~30+), which biases an unweighted (kernel=None) fit away
    // from identity.
    for (int i = 0; i < 5; ++i) {
        source.emplace_back(20.0f + static_cast<float>(i), 20.0f, 20.0f);
    }

    ICPRegistration icp;

    ICPRegistration::Result withoutKernel;
    ASSERT_TRUE(icp.align(source, target, withoutKernel, 5, 1.0e-8f, 0.0f, ICPRegistration::RobustKernel::None));

    ICPRegistration::Result withKernel;
    ASSERT_TRUE(icp.align(source, target, withKernel, 5, 1.0e-8f, 0.0f, ICPRegistration::RobustKernel::Tukey, 0.5f));

    // delta=0.5 is far below the outliers' ~30-unit residual, so Tukey weights them to ~0
    // and the fit stays close to identity; without a kernel they pull the translation away.
    EXPECT_LT(glm::length(withKernel.translation), glm::length(withoutKernel.translation));
    EXPECT_LT(glm::length(withKernel.translation), 0.1f);
}

TEST(ICPRegistrationTest, PointToPlane_FailsWithMismatchedNormalsSize)
{
    ICPRegistration icp;
    ICPRegistration::Result result;
    std::vector<Vector3df> pts = { Vector3df(0, 0, 0), Vector3df(1, 0, 0), Vector3df(0, 1, 0) };
    std::vector<Vector3df> normals = { Vector3df(0, 0, 1) }; // wrong size
    EXPECT_FALSE(icp.alignPointToPlane(pts, pts, normals, result));
}

TEST(ICPRegistrationTest, PointToPlane_FailsWithEmptyInput)
{
    ICPRegistration icp;
    ICPRegistration::Result result;
    EXPECT_FALSE(icp.alignPointToPlane({}, { Vector3df(0, 0, 0) }, { Vector3df(0, 0, 1) }, result));
}

TEST(ICPRegistrationTest, PointToPlane_RecoversSmallTranslation)
{
    const auto target = makeSpherePoints(300, 1.0f, 21);
    std::vector<Vector3df> targetNormals;
    targetNormals.reserve(target.size());
    for (const auto& p : target) targetNormals.push_back(glm::normalize(p)); // sphere centered at origin

    const Vector3df offset(0.02f, -0.01f, 0.015f);
    const auto source = translate(target, offset);

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.alignPointToPlane(source, target, targetNormals, result, 50, 1.0e-8f));

    for (size_t i = 0; i < source.size(); i += 17) {
        const auto aligned = ICPRegistration::transformPoint(result, source[i]);
        EXPECT_LT(nearestDistance(aligned, target), 0.05f);
    }
}

// F-3: a ProgressReporter that always continues must not change the alignment
// result (same setup/assertions as RecoversExactTranslation), and progress
// must be reported at least once per completed iteration, non-decreasing.
TEST(ICPRegistrationTest, ProgressReporterReportsMonotonicallyWithoutCancelling)
{
    const auto target = makeSpherePoints(300, 1.0f, 42);
    const Vector3df offset(0.05f, -0.03f, 0.02f);
    const auto source = translate(target, offset);

    std::vector<float> reported;
    ProgressReporter reporter;
    reporter.callback = [&](float progress) -> bool {
        reported.push_back(progress);
        return true;
    };

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(source, target, result, 50, 1.0e-8f, 0.0f,
                           ICPRegistration::RobustKernel::None, 1.0f, false, reporter));

    EXPECT_NEAR(result.fitness, 0.0f, 1.0e-4f);
    for (size_t i = 0; i < source.size(); ++i) {
        const auto aligned = ICPRegistration::transformPoint(result, source[i]);
        EXPECT_NEAR(aligned.x, target[i].x, 1.0e-2f);
        EXPECT_NEAR(aligned.y, target[i].y, 1.0e-2f);
        EXPECT_NEAR(aligned.z, target[i].z, 1.0e-2f);
    }

    ASSERT_FALSE(reported.empty());
    for (size_t i = 1; i < reported.size(); ++i) {
        EXPECT_GE(reported[i], reported[i - 1]);
    }
}

// F-3: returning false from the callback must stop ICP well before
// maxIterations, while still returning a valid (partial) Result reflecting
// the transform accumulated up through the last completed iteration. Cancels
// on the very first call: since `hasFitness` starts false, the convergence
// check at iter=0 can never short-circuit before that first report(), so
// exactly one completed iteration (and one callback call) is guaranteed
// regardless of how quickly this dataset would otherwise have converged.
TEST(ICPRegistrationTest, ProgressReporterCancellationStopsIterationEarly)
{
    const auto target = makeSpherePoints(300, 1.0f, 42);
    const Vector3df offset(0.05f, -0.03f, 0.02f);
    const auto source = translate(target, offset);

    int callCount = 0;
    ProgressReporter reporter;
    reporter.callback = [&](float /*progress*/) -> bool {
        ++callCount;
        return false; // cancel immediately after the 1st completed iteration
    };

    ICPRegistration icp;
    ICPRegistration::Result result;
    ASSERT_TRUE(icp.align(source, target, result, /*maxIterations=*/50, 1.0e-8f, 0.0f,
                           ICPRegistration::RobustKernel::None, 1.0f, false, reporter));

    EXPECT_EQ(callCount, 1);
    EXPECT_EQ(result.iterations, 1);
    EXPECT_FALSE(result.converged);
}
