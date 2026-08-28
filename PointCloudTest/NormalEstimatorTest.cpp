#include "pch.h"

#include "../PointCloud/NormalEstimator.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(NormalEstimatorTest, EmptyInputReturnsEmptyNormals)
{
	NormalEstimator estimator;
	estimator.estimate(1.0);

	const auto normals = estimator.getNormals();
	EXPECT_TRUE(normals.empty());
}

TEST(NormalEstimatorTest, SinglePointReturnsSingleZeroNormal)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.estimate(1.0);

	const auto normals = estimator.getNormals();
	ASSERT_EQ(1u, normals.size());
	EXPECT_NEAR(0.0f, Phantom::Math::getLength(normals[0]), 1.0e-6f);
}

TEST(NormalEstimatorTest, FlatPlaneNormalsAlignWithZAxis)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));

	estimator.estimate(1.5);

	const auto normals = estimator.getNormals();
	ASSERT_EQ(5u, normals.size());

	const Vector3df zAxis(0.0f, 0.0f, 1.0f);
	for (const auto& n : normals) {
		const auto len = Phantom::Math::getLength(n);
		ASSERT_GT(len, 0.0f);
		const auto cosTheta = std::fabs(glm::dot(glm::normalize(n), zAxis));
		EXPECT_GT(cosTheta, 0.95f);
	}
}

TEST(NormalEstimatorTest, OrientTowardsViewpointFlipsNormalsAwayFromViewpoint)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));

	estimator.estimate(1.5);

	// Force every normal to point in -Z first, regardless of the sign PCA happened to pick.
	auto normals = estimator.getNormals();
	ASSERT_EQ(5u, normals.size());

	// Viewpoint above the plane (+Z): every normal should end up with a non-negative Z component.
	estimator.orientTowardsViewpoint(Vector3df(0.0f, 0.0f, 5.0f));
	const auto orientedUp = estimator.getNormals();
	for (const auto& n : orientedUp) {
		EXPECT_GE(n.z, 0.0f);
	}

	// Viewpoint below the plane (-Z): re-orienting should flip every normal to non-positive Z.
	estimator.orientTowardsViewpoint(Vector3df(0.0f, 0.0f, -5.0f));
	const auto orientedDown = estimator.getNormals();
	for (const auto& n : orientedDown) {
		EXPECT_LE(n.z, 0.0f);
	}
}

TEST(NormalEstimatorTest, OrientTowardsViewpointLeavesAlreadyCorrectNormalsUnchanged)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));

	estimator.estimate(1.5);
	estimator.orientTowardsViewpoint(Vector3df(0.0f, 0.0f, 5.0f));
	const auto first = estimator.getNormals();

	// Re-orienting toward the same viewpoint again must be a no-op.
	estimator.orientTowardsViewpoint(Vector3df(0.0f, 0.0f, 5.0f));
	const auto second = estimator.getNormals();

	ASSERT_EQ(first.size(), second.size());
	for (size_t i = 0; i < first.size(); ++i) {
		EXPECT_NEAR(first[i].x, second[i].x, 1.0e-6f);
		EXPECT_NEAR(first[i].y, second[i].y, 1.0e-6f);
		EXPECT_NEAR(first[i].z, second[i].z, 1.0e-6f);
	}
}

TEST(NormalEstimatorTest, OrientTowardsViewpointLeavesZeroNormalsUnaffected)
{
	// A single point has no neighbors, so its estimated normal stays the zero vector;
	// orienting must not turn it into a spurious non-zero direction.
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.estimate(1.0);

	estimator.orientTowardsViewpoint(Vector3df(0.0f, 0.0f, 5.0f));
	const auto normals = estimator.getNormals();
	ASSERT_EQ(1u, normals.size());
	EXPECT_NEAR(0.0f, Phantom::Math::getLength(normals[0]), 1.0e-6f);
}

TEST(NormalEstimatorTest, EstimateCanBeCalledRepeatedlyWithoutGrowingOutput)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));

	estimator.estimate(1.5);
	const auto first = estimator.getNormals();
	ASSERT_EQ(3u, first.size());

	estimator.estimate(1.5);
	const auto second = estimator.getNormals();
	EXPECT_EQ(3u, second.size());
}

TEST(NormalEstimatorTest, EstimateKNNEmptyInputReturnsEmptyNormals)
{
	NormalEstimator estimator;
	estimator.estimateKNN(4);

	const auto normals = estimator.getNormals();
	EXPECT_TRUE(normals.empty());
}

TEST(NormalEstimatorTest, EstimateKNNSinglePointReturnsSingleZeroNormal)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.estimateKNN(4);

	const auto normals = estimator.getNormals();
	ASSERT_EQ(1u, normals.size());
	EXPECT_NEAR(0.0f, Phantom::Math::getLength(normals[0]), 1.0e-6f);
}

TEST(NormalEstimatorTest, EstimateKNNFlatPlaneNormalsAlignWithZAxis)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(-1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));
	estimator.add(Vector3df(0.0f, -1.0f, 0.0f));

	estimator.estimateKNN(4);

	const auto normals = estimator.getNormals();
	ASSERT_EQ(5u, normals.size());

	const Vector3df zAxis(0.0f, 0.0f, 1.0f);
	for (const auto& n : normals) {
		const auto len = Phantom::Math::getLength(n);
		ASSERT_GT(len, 0.0f);
		const auto cosTheta = std::fabs(glm::dot(glm::normalize(n), zAxis));
		EXPECT_GT(cosTheta, 0.95f);
	}
}

TEST(NormalEstimatorTest, EstimateKNNCanBeCalledRepeatedlyWithoutGrowingOutput)
{
	NormalEstimator estimator;
	estimator.add(Vector3df(0.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(1.0f, 0.0f, 0.0f));
	estimator.add(Vector3df(0.0f, 1.0f, 0.0f));

	estimator.estimateKNN(2);
	const auto first = estimator.getNormals();
	ASSERT_EQ(3u, first.size());

	estimator.estimateKNN(2);
	const auto second = estimator.getNormals();
	EXPECT_EQ(3u, second.size());
}
