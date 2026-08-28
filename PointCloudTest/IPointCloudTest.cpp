#include "pch.h"

#include "../PointCloud/PointCloud.h"
#include "../PointCloud/ColoredPointCloud.h"
#include "../PointCloud/NormalEstimator.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {

// A minimal stand-in for the existing single-point `add()`-style algorithms (NormalEstimator,
// DownSampler, ...) that feedPositions() is meant to work with, so the FeedPositionsTest cases
// don't depend on IPointCloud's own (differently-named) addPosition().
struct RecordingConsumer {
	std::vector<Vector3df> received;
	void add(const Vector3df& position) { received.push_back(position); }
};

} // namespace

TEST(PointCloudTest, EmptySizeIsZero)
{
	PointCloud cloud;
	EXPECT_EQ(0u, cloud.size());
	EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudTest, AddPositionAndGetPosition)
{
	PointCloud cloud;
	cloud.addPosition(Vector3df(1.0f, 2.0f, 3.0f));
	cloud.addPosition(Vector3df(-4.0f, 5.0f, 0.5f));

	ASSERT_EQ(2u, cloud.size());
	EXPECT_FALSE(cloud.empty());
	EXPECT_FLOAT_EQ(1.0f, cloud.getPosition(0).x);
	EXPECT_FLOAT_EQ(5.0f, cloud.getPosition(1).y);
}

TEST(PointCloudTest, ConstructFromExistingPositions)
{
	std::vector<Vector3df> positions = { Vector3df(0, 0, 0), Vector3df(1, 1, 1) };
	PointCloud cloud(positions);

	ASSERT_EQ(2u, cloud.size());
	EXPECT_EQ(2u, cloud.getPositions().size());
}

TEST(PointCloudTest, SetPositionsReplacesArray)
{
	PointCloud cloud;
	cloud.addPosition(Vector3df(0, 0, 0));

	cloud.setPositions({ Vector3df(1, 2, 3), Vector3df(4, 5, 6), Vector3df(7, 8, 9) });

	ASSERT_EQ(3u, cloud.size());
	EXPECT_FLOAT_EQ(4.0f, cloud.getPositions()[1].x);
}

TEST(PointCloudTest, BoundingBoxComesFromIPointCloud)
{
	PointCloud cloud;
	cloud.addPosition(Vector3df(1.0f, 2.0f, 3.0f));
	cloud.addPosition(Vector3df(-4.0f, 5.0f, 0.5f));
	cloud.addPosition(Vector3df(2.5f, -1.0f, 10.0f));

	const auto bb = cloud.getBoundingBox();
	EXPECT_FLOAT_EQ(-4.0f, bb.getMin().x);
	EXPECT_FLOAT_EQ(-1.0f, bb.getMin().y);
	EXPECT_FLOAT_EQ(0.5f, bb.getMin().z);
	EXPECT_FLOAT_EQ(2.5f, bb.getMax().x);
	EXPECT_FLOAT_EQ(5.0f, bb.getMax().y);
	EXPECT_FLOAT_EQ(10.0f, bb.getMax().z);
}

TEST(PointCloudTest, IsUsableAsIPointCloud)
{
	PointCloud cloud;
	cloud.addPosition(Vector3df(1.0f, 0.0f, 0.0f));

	IPointCloud& asInterface = cloud;
	EXPECT_EQ(1u, asInterface.size());
	EXPECT_FLOAT_EQ(1.0f, asInterface.getPosition(0).x);

	asInterface.addPosition(Vector3df(9.0f, 0.0f, 0.0f));
	EXPECT_EQ(2u, cloud.size());

	asInterface.setPositions({ Vector3df(3.0f, 0.0f, 0.0f) });
	EXPECT_EQ(1u, cloud.size());
	EXPECT_FLOAT_EQ(3.0f, cloud.getPosition(0).x);
}

TEST(ColoredPointCloudTest, EmptySizeIsZero)
{
	ColoredPointCloud cloud;
	EXPECT_EQ(0u, cloud.size());
	EXPECT_TRUE(cloud.empty());
	EXPECT_FALSE(cloud.hasColors());
	EXPECT_FALSE(cloud.hasNormals());
	EXPECT_FALSE(cloud.hasScalars());
}

TEST(ColoredPointCloudTest, AddPositionColorNormalScalar)
{
	ColoredPointCloud cloud;
	cloud.addPosition(Vector3df(1.0f, 2.0f, 3.0f));
	cloud.addColor(Vector3df(1.0f, 0.5f, 0.25f));
	cloud.addNormal(Vector3df(0.0f, 0.0f, 1.0f));
	cloud.addScalar(3.14f);

	ASSERT_EQ(1u, cloud.size());
	EXPECT_TRUE(cloud.hasColors());
	EXPECT_TRUE(cloud.hasNormals());
	EXPECT_TRUE(cloud.hasScalars());
	EXPECT_FLOAT_EQ(3.0f, cloud.getPosition(0).z);
	EXPECT_FLOAT_EQ(0.5f, cloud.getColor(0).y);
	EXPECT_FLOAT_EQ(1.0f, cloud.getNormal(0).z);
	EXPECT_FLOAT_EQ(3.14f, cloud.getScalar(0));
}

TEST(ColoredPointCloudTest, GetPositionsAndSetPositions)
{
	ColoredPointCloud cloud;
	cloud.addPosition(Vector3df(1, 2, 3));
	ASSERT_EQ(1u, cloud.getPositions().size());

	cloud.setPositions({ Vector3df(4, 5, 6), Vector3df(7, 8, 9) });
	ASSERT_EQ(2u, cloud.size());
	EXPECT_FLOAT_EQ(7.0f, cloud.getPositions()[1].x);
	// setPositions() only touches the position array; colors/normals/scalars are untouched.
	EXPECT_FALSE(cloud.hasColors());
}

TEST(ColoredPointCloudTest, WrapsExistingPointCloudColoredData)
{
	PointCloudColoredData data;
	data.positions = { Vector3df(1, 2, 3), Vector3df(4, 5, 6) };
	data.colors = { Vector3df(1, 0, 0), Vector3df(0, 1, 0) };

	ColoredPointCloud cloud(data);

	ASSERT_EQ(2u, cloud.size());
	EXPECT_TRUE(cloud.hasColors());
	EXPECT_FLOAT_EQ(4.0f, cloud.getPosition(1).x);
	EXPECT_FLOAT_EQ(1.0f, cloud.getColor(1).y);
	// The wrapped data is reachable unchanged for APIs that still take
	// PointCloudColoredData directly (e.g. savePointCloud()).
	EXPECT_EQ(2u, cloud.getData().size());
}

TEST(ColoredPointCloudTest, IsUsableAsIPointCloud)
{
	ColoredPointCloud cloud;
	cloud.addPosition(Vector3df(2.0f, 0.0f, 0.0f));

	const IPointCloud& asInterface = cloud;
	EXPECT_EQ(1u, asInterface.size());
	EXPECT_FLOAT_EQ(2.0f, asInterface.getPosition(0).x);
}

// feedPositions() should behave identically to calling add() directly for each point, so
// reuse the same flat-plane setup as NormalEstimatorTest.FlatPlaneNormalsAlignWithZAxis.
TEST(FeedPositionsTest, FeedsPointCloudIntoNormalEstimator)
{
	PointCloud cloud;
	cloud.addPosition(Vector3df(0.0f, 0.0f, 0.0f));
	cloud.addPosition(Vector3df(1.0f, 0.0f, 0.0f));
	cloud.addPosition(Vector3df(-1.0f, 0.0f, 0.0f));
	cloud.addPosition(Vector3df(0.0f, 1.0f, 0.0f));
	cloud.addPosition(Vector3df(0.0f, -1.0f, 0.0f));

	NormalEstimator estimator;
	feedPositions(estimator, cloud);
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

TEST(FeedPositionsTest, FeedsColoredPointCloudPositionsOnly)
{
	ColoredPointCloud cloud;
	cloud.addPosition(Vector3df(1.0f, 0.0f, 0.0f));
	cloud.addPosition(Vector3df(2.0f, 0.0f, 0.0f));
	cloud.addColor(Vector3df(1.0f, 1.0f, 1.0f)); // colors not consumed by feedPositions()

	RecordingConsumer consumer;
	feedPositions(consumer, cloud);

	ASSERT_EQ(2u, consumer.received.size());
	EXPECT_FLOAT_EQ(1.0f, consumer.received[0].x);
	EXPECT_FLOAT_EQ(2.0f, consumer.received[1].x);
}
