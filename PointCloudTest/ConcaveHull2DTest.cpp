#include "pch.h"
#include "gtest/gtest.h"

#include "../PointCloud/ConcaveHull2D.h"
#include "../PointCloud/ConvexHull2D.h"
#include "../../CGLib/Math/Vector3d.h"
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

void addRange(std::vector<Vector3df>& points, float x0, float y0, float x1, float y1, float step)
{
	const float dx = x1 - x0, dy = y1 - y0;
	const float length = std::sqrt(dx * dx + dy * dy);
	const int n = static_cast<int>(length / step);
	for (int i = 0; i <= n; ++i) {
		const float t = static_cast<float>(i) / static_cast<float>(n);
		points.emplace_back(x0 + dx * t, y0 + dy * t, 0.0f);
	}
}

// Densely-sampled boundary of a "U"/staple shape: a 6x6 square with a 2-wide, 4-tall rectangular
// notch cut out of the top-middle. Sampled at 0.5 spacing so that, at small k, each point's
// nearest neighbors are its immediate boundary neighbors rather than points across the notch gap
// -- this is what makes a small k reliably trace the concavity instead of shortcutting across it.
std::vector<Vector3df> makeUShapeBoundary()
{
	std::vector<Vector3df> points;
	addRange(points, 0, 0, 6, 0, 0.5f); // bottom edge
	addRange(points, 6, 0, 6, 6, 0.5f); // right edge
	addRange(points, 6, 6, 4, 6, 0.5f); // top-right segment
	addRange(points, 4, 6, 4, 2, 0.5f); // notch right wall (down)
	addRange(points, 4, 2, 2, 2, 0.5f); // notch bottom
	addRange(points, 2, 2, 2, 6, 0.5f); // notch left wall (up)
	addRange(points, 2, 6, 0, 6, 0.5f); // top-left segment
	addRange(points, 0, 6, 0, 0, 0.5f); // left edge
	return points;
}

} // namespace

TEST(ConcaveHull2DTest, FewerThanThreePointsFails)
{
	ConcaveHull2D hull;
	hull.add(Vector3df(0, 0, 0));
	hull.add(Vector3df(1, 0, 0));
	EXPECT_FALSE(hull.compute());
	EXPECT_TRUE(hull.getHullPoints().empty());
	EXPECT_FLOAT_EQ(0.0f, hull.getArea());
}

TEST(ConcaveHull2DTest, ThreePointsFormTheirOwnTriangle)
{
	ConcaveHull2D hull;
	hull.add(Vector3df(0, 0, 0));
	hull.add(Vector3df(4, 0, 0));
	hull.add(Vector3df(0, 3, 0));
	ASSERT_TRUE(hull.compute());
	EXPECT_EQ(3u, hull.getHullPoints().size());
	EXPECT_NEAR(6.0f, hull.getArea(), 1.0e-4f);
}

TEST(ConcaveHull2DTest, DuplicatePointsAreDeduped)
{
	ConcaveHull2D hull;
	hull.add(Vector3df(-2, -2, 0));
	hull.add(Vector3df(-2, -2, 0)); // exact duplicate
	hull.add(Vector3df(2, -2, 0));
	hull.add(Vector3df(2, 2, 0));
	hull.add(Vector3df(-2, 2, 0));
	ASSERT_TRUE(hull.compute());
	EXPECT_EQ(4u, hull.getHullPoints().size());
}

TEST(ConcaveHull2DTest, ConvexSquareReducesToConvexHull)
{
	ConcaveHull2D hull;
	hull.add(Vector3df(-2, -2, 0));
	hull.add(Vector3df(2, -2, 0));
	hull.add(Vector3df(2, 2, 0));
	hull.add(Vector3df(-2, 2, 0));

	ASSERT_TRUE(hull.compute(3));
	const auto points = hull.getHullPoints();
	ASSERT_EQ(4u, points.size());
	EXPECT_NEAR(16.0f, hull.getArea(), 1.0e-4f);
}

TEST(ConcaveHull2DTest, HullVerticesAreInCounterClockwiseOrder)
{
	ConcaveHull2D hull;
	hull.add(Vector3df(-2, -2, 0));
	hull.add(Vector3df(2, -2, 0));
	hull.add(Vector3df(2, 2, 0));
	hull.add(Vector3df(-2, 2, 0));

	ASSERT_TRUE(hull.compute(3));
	const auto points = hull.getHullPoints();
	ASSERT_EQ(4u, points.size());

	double signedArea = 0.0;
	for (size_t i = 0; i < points.size(); ++i) {
		const auto& p0 = points[i];
		const auto& p1 = points[(i + 1) % points.size()];
		signedArea += static_cast<double>(p0.x) * p1.y - static_cast<double>(p1.x) * p0.y;
	}
	EXPECT_GT(signedArea, 0.0);
}

TEST(ConcaveHull2DTest, UShapedBoundaryProducesTighterHullThanConvexHull)
{
	// With few, widely-scattered points, a straight-line polygon between the extremal points
	// already contains everything, so nothing forces the trace to detour through a concavity
	// (this is correct: the k-nearest algorithm doesn't minimize area, it just accepts the first
	// valid trace). A dense boundary sampling is what makes small k reliably hug the shape,
	// which is also how this algorithm is actually meant to be used against real scan data.
	const auto points = makeUShapeBoundary();

	ConcaveHull2D concave;
	for (const auto& p : points) concave.add(p);
	ASSERT_TRUE(concave.compute(3));

	ConvexHull2D convex;
	for (const auto& p : points) convex.add(p);
	ASSERT_TRUE(convex.compute());

	EXPECT_LT(concave.getArea(), convex.getArea());
	EXPECT_GT(concave.getHullPoints().size(), 4u);
}
