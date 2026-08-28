#include "pch.h"
#include "../PointCloud/GlobalRegistration.h"
#include "../PointCloud/FPFHEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

// A right-angle corner made of two differently-sized/spaced planar patches sharing an edge
// along the X axis (Face A in the XY plane, Face B in the XZ plane). Unlike a sphere or a
// single plane, this has no continuous rotational symmetry and no reflection symmetry (thanks
// to the mismatched face extents), so there is exactly one rigid pose that aligns it onto a
// transformed copy of itself.
std::pair<std::vector<Vector3df>, std::vector<Vector3df>> makeCornerCloud()
{
	std::vector<Vector3df> positions;
	std::vector<Vector3df> normals;

	// Face A: XY plane (z=0), normal +Z.
	for (int ix = 0; ix <= 8; ++ix) {
		for (int iy = 0; iy <= 6; ++iy) {
			positions.emplace_back(0.25f * ix, 0.25f * iy, 0.0f);
			normals.emplace_back(0.0f, 0.0f, 1.0f);
		}
	}
	// Face B: XZ plane (y=0), normal +Y. Different span/density than Face A so the two faces
	// aren't interchangeable under any rigid transform.
	for (int ix = 0; ix <= 8; ++ix) {
		for (int iz = 0; iz <= 4; ++iz) {
			positions.emplace_back(0.25f * ix, 0.0f, 0.2f * iz);
			normals.emplace_back(0.0f, 1.0f, 0.0f);
		}
	}

	return { positions, normals };
}

std::vector<Vector3df> translate(const std::vector<Vector3df>& pts, const Vector3df& offset)
{
	std::vector<Vector3df> result;
	result.reserve(pts.size());
	for (const auto& p : pts) result.push_back(p + offset);
	return result;
}

// Rotation (Rodrigues' formula) about a fixed axis not aligned with either face normal or the
// shared edge, so the test doesn't rely on an axis-aligned special case.
Matrix3df makeTestRotation(float angleRad)
{
	const Vector3df axis = glm::normalize(Vector3df(0.3f, 0.5f, 0.8f));
	const float c = std::cos(angleRad);
	const float s = std::sin(angleRad);
	const float t = 1.0f - c;
	return Matrix3df(
		t * axis.x * axis.x + c, t * axis.x * axis.y - s * axis.z, t * axis.x * axis.z + s * axis.y,
		t * axis.x * axis.y + s * axis.z, t * axis.y * axis.y + c, t * axis.y * axis.z - s * axis.x,
		t * axis.x * axis.z - s * axis.y, t * axis.y * axis.z + s * axis.x, t * axis.z * axis.z + c);
}

std::vector<Vector3df> rotateAll(const std::vector<Vector3df>& vecs, const Matrix3df& r)
{
	std::vector<Vector3df> result;
	result.reserve(vecs.size());
	for (const auto& v : vecs) result.push_back(r * v);
	return result;
}

float nearestDistance(const Vector3df& p, const std::vector<Vector3df>& cloud)
{
	float best = std::numeric_limits<float>::max();
	for (const auto& q : cloud) best = std::min(best, glm::length(q - p));
	return best;
}

} // namespace

TEST(GlobalRegistrationTest, FailsWithMismatchedFeatureSizes)
{
	GlobalRegistration reg;
	GlobalRegistration::Result result;
	std::vector<Vector3df> pts = { Vector3df(0, 0, 0), Vector3df(1, 0, 0), Vector3df(0, 1, 0) };
	std::vector<FPFHEstimator::Histogram> mismatched(2);
	EXPECT_FALSE(reg.align(pts, mismatched, pts, mismatched, result));
}

TEST(GlobalRegistrationTest, FailsWithFewerPointsThanSampleSize)
{
	GlobalRegistration reg;
	GlobalRegistration::Result result;
	std::vector<Vector3df> pts = { Vector3df(0, 0, 0), Vector3df(1, 0, 0) }; // < default sampleSize (3)
	std::vector<FPFHEstimator::Histogram> feats(2);
	EXPECT_FALSE(reg.align(pts, feats, pts, feats, result));
}

TEST(GlobalRegistrationTest, RecoversRotationAndTranslationFromUnknownPose)
{
	const auto [targetPos, targetNormals] = makeCornerCloud();

	FPFHEstimator targetEst;
	for (size_t i = 0; i < targetPos.size(); ++i) targetEst.add(targetPos[i], targetNormals[i]);
	ASSERT_TRUE(targetEst.estimate(10));

	const Matrix3df trueRotation = makeTestRotation(0.9f);
	const Vector3df trueTranslation(1.5f, -2.0f, 0.7f);
	const auto sourcePos = translate(rotateAll(targetPos, trueRotation), trueTranslation);
	const auto sourceNormals = rotateAll(targetNormals, trueRotation);

	FPFHEstimator sourceEst;
	for (size_t i = 0; i < sourcePos.size(); ++i) sourceEst.add(sourcePos[i], sourceNormals[i]);
	ASSERT_TRUE(sourceEst.estimate(10));

	GlobalRegistration reg(1234);
	GlobalRegistration::Result result;
	ASSERT_TRUE(reg.align(
		sourcePos, sourceEst.getHistograms(),
		targetPos, targetEst.getHistograms(),
		result,
		3000, 0.1f, 3, 0.2f, 10));

	// The recovered transform doesn't need to reproduce the exact index-level correspondence --
	// just land every transformed source point close to *some* target point, the same way
	// ICPRegistrationTest checks rotation+translation recovery.
	size_t sampled = 0;
	size_t closeCount = 0;
	for (size_t i = 0; i < sourcePos.size(); i += 5) {
		const Vector3df aligned = result.rotation * sourcePos[i] + result.translation;
		++sampled;
		if (nearestDistance(aligned, targetPos) < 0.15f) ++closeCount;
	}
	EXPECT_GT(closeCount, sampled / 2);
}

TEST(GlobalRegistrationTest, FailsForScaleMismatchedClouds)
{
	// GlobalRegistration is rigid-only (no scale estimation, unlike ICPRegistration::align's
	// optional Umeyama step): a cloud and a uniformly-scaled copy of itself share (nearly)
	// identical FPFH descriptors per point (FPFH is built from angular relationships only, see
	// FPFHEstimatorTest.TranslationInvariance for the same invariance under translation), so
	// feature matching alone finds correct-looking correspondences here -- but no rigid
	// transform can bring a 50x-scaled copy within a tight maxCorrespondenceDistance, so RANSAC's
	// geometric consistency checks must still reject every sample.
	std::vector<Vector3df> target = {
		Vector3df(1, 0, 0), Vector3df(0, 1, 0), Vector3df(0, 0, 1), Vector3df(1, 1, 1), Vector3df(-1, 1, 0)
	};
	std::vector<Vector3df> targetNormals;
	for (const auto& p : target) targetNormals.push_back(glm::normalize(p));

	std::vector<Vector3df> source;
	for (const auto& p : target) source.push_back(p * 50.0f);
	const std::vector<Vector3df>& sourceNormals = targetNormals; // normalize(p*50) == normalize(p)

	FPFHEstimator targetEst, sourceEst;
	for (size_t i = 0; i < target.size(); ++i) targetEst.add(target[i], targetNormals[i]);
	for (size_t i = 0; i < source.size(); ++i) sourceEst.add(source[i], sourceNormals[i]);
	ASSERT_TRUE(targetEst.estimate(3));
	ASSERT_TRUE(sourceEst.estimate(3));

	GlobalRegistration reg(7);
	GlobalRegistration::Result result;
	EXPECT_FALSE(reg.align(
		source, sourceEst.getHistograms(),
		target, targetEst.getHistograms(),
		result,
		500, 0.05f, 3, 0.15f, 3));
}
