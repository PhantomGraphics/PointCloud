#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/ConvexHull2D.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

TEST(ConvexHull2DTest, FewerThanThreePointsFails)
{
	ConvexHull2D hull;
	hull.add(Vector3df(0, 0, 0));
	hull.add(Vector3df(1, 0, 0));
	EXPECT_FALSE(hull.compute());
	EXPECT_TRUE(hull.getHullPoints().empty());
	EXPECT_FLOAT_EQ(0.0f, hull.getArea());
}

TEST(ConvexHull2DTest, AllCollinearPointsFails)
{
	ConvexHull2D hull;
	hull.add(Vector3df(0, 0, 0));
	hull.add(Vector3df(1, 0, 0));
	hull.add(Vector3df(2, 0, 0));
	hull.add(Vector3df(3, 0, 0));
	EXPECT_FALSE(hull.compute());
	EXPECT_TRUE(hull.getHullPoints().empty());
}

TEST(ConvexHull2DTest, SquareWithInteriorAndEdgePointsProducesFourVertexHull)
{
	ConvexHull2D hull;
	// The 4 corners of a 4x4 square.
	hull.add(Vector3df(-2, -2, 5.0f)); // z is arbitrary/ignored
	hull.add(Vector3df(2, -2, 0.0f));
	hull.add(Vector3df(2, 2, 0.0f));
	hull.add(Vector3df(-2, 2, 0.0f));
	// An interior point -- must not appear on the hull.
	hull.add(Vector3df(0, 0, 0.0f));
	// A point exactly on the bottom edge -- collinear with two hull vertices, must be dropped.
	hull.add(Vector3df(0, -2, 0.0f));

	ASSERT_TRUE(hull.compute());
	const auto points = hull.getHullPoints();
	ASSERT_EQ(4u, points.size());
	EXPECT_NEAR(16.0f, hull.getArea(), 1.0e-4f);

	for (const auto& p : points) {
		const bool isCorner =
			(std::abs(p.x) == 2.0f && std::abs(p.y) == 2.0f);
		EXPECT_TRUE(isCorner);
		EXPECT_FLOAT_EQ(0.0f, p.z); // z is reset, not carried over
	}
}

TEST(ConvexHull2DTest, HullVerticesAreInCounterClockwiseOrder)
{
	ConvexHull2D hull;
	hull.add(Vector3df(-2, -2, 0));
	hull.add(Vector3df(2, -2, 0));
	hull.add(Vector3df(2, 2, 0));
	hull.add(Vector3df(-2, 2, 0));

	ASSERT_TRUE(hull.compute());
	const auto points = hull.getHullPoints();
	ASSERT_EQ(4u, points.size());

	// Signed area (shoelace, unscaled) is positive for a CCW polygon.
	double signedArea = 0.0;
	for (size_t i = 0; i < points.size(); ++i) {
		const auto& p0 = points[i];
		const auto& p1 = points[(i + 1) % points.size()];
		signedArea += static_cast<double>(p0.x) * p1.y - static_cast<double>(p1.x) * p0.y;
	}
	EXPECT_GT(signedArea, 0.0);
}

TEST(ConvexHull2DTest, TriangleProducesCorrectArea)
{
	ConvexHull2D hull;
	hull.add(Vector3df(0, 0, 0));
	hull.add(Vector3df(4, 0, 0));
	hull.add(Vector3df(0, 3, 0));

	ASSERT_TRUE(hull.compute());
	EXPECT_NEAR(6.0f, hull.getArea(), 1.0e-4f); // 0.5*base*height = 0.5*4*3
}
