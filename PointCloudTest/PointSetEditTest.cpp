#include "pch.h"

#include "../Core/PointSet.h"
#include "../Core/PointSetEdit.h"
#include "../Core/PointSetGenerate.h"
#include "../Core/PointSetPick.h"

#include "../../CGLib/ThirdParty/glm/glm/gtc/matrix_transform.hpp"

#include <cmath>

using namespace Phantom;
using namespace Phantom::PC;

namespace {

	PointSet makeSet(size_t n, bool withNormals = false)
	{
		PointSet s;
		for (size_t i = 0; i < n; ++i) {
			const float f = static_cast<float>(i);
			s.positions.emplace_back(f, 0.f, 0.f);
			s.colors.emplace_back(0.1f * f, 0.5f, 0.25f);
			s.radii.push_back(0.1f + 0.01f * f);
			if (withNormals) s.normals.emplace_back(0.f, 1.f, 0.f);
		}
		return s;
	}

	// Camera at (0,0,5) looking down -Z, 90 degree vertical field of view, 200x200 pixels: pixel (100,100) is straight ahead.
	PC::PointPickView frontView()
	{
		PC::PointPickView v;
		v.view = glm::lookAt(glm::vec3(0.f, 0.f, 5.f), glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f));
		v.proj = glm::perspective(glm::radians(90.f), 1.f, 0.1f, 100.f);
		v.proj[1][1] *= -1.f;   // the Vulkan Y flip the renderer applies
		v.width = 200.f;
		v.height = 200.f;
		return v;
	}

	PointSet depthColumn()
	{
		// Three points on the view axis at z = 0 (front), -3 and -6, plus one off to the side.
		PointSet s;
		for (float z : { 0.f, -3.f, -6.f }) {
			s.positions.emplace_back(0.f, 0.f, z);
			s.colors.emplace_back(1.f, 1.f, 1.f);
			s.radii.push_back(0.2f);
		}
		s.positions.emplace_back(2.f, 0.f, 0.f);
		s.colors.emplace_back(1.f, 1.f, 1.f);
		s.radii.push_back(0.2f);
		return s;
	}

}

// ---- selection lists --------------------------------------------------------------------------------------------------------

TEST(PointSetEditTest, NormalizeSortsDeduplicatesAndDropsOutOfRange)
{
	EXPECT_EQ((PointIndexList{ 1, 3, 4 }), normalizeSelection({ 4, 1, 3, 1, 9, 4 }, 5));
	EXPECT_TRUE(normalizeSelection({ 7 }, 5).empty());
	EXPECT_TRUE(normalizeSelection({}, 5).empty());
}

TEST(PointSetEditTest, CombineModes)
{
	const PointIndexList current{ 1, 2, 3 }, hits{ 3, 4 };
	EXPECT_EQ((PointIndexList{ 3, 4 }), combineSelection(current, hits, SelectMode::Replace));
	EXPECT_EQ((PointIndexList{ 1, 2, 3, 4 }), combineSelection(current, hits, SelectMode::Add));
	EXPECT_EQ((PointIndexList{ 1, 2 }), combineSelection(current, hits, SelectMode::Remove));
	EXPECT_EQ((PointIndexList{ 1, 2, 4 }), combineSelection(current, hits, SelectMode::Toggle));
}

TEST(PointSetEditTest, AllAndInvert)
{
	EXPECT_EQ((PointIndexList{ 0, 1, 2, 3 }), allIndices(4));
	EXPECT_EQ((PointIndexList{ 0, 2 }), invertSelection({ 1, 3 }, 4));
	EXPECT_EQ(allIndices(3), invertSelection({}, 3));
	EXPECT_TRUE(invertSelection(allIndices(3), 3).empty());
}

TEST(PointSetEditTest, CentroidAndBoundsOfASelection)
{
	const PointSet s = makeSet(5);
	const auto c = selectionCentroid(s, { 1, 3 });
	ASSERT_TRUE(c.has_value());
	EXPECT_FLOAT_EQ(2.f, c->x);
	const auto b = selectionBounds(s, { 1, 3 });
	ASSERT_TRUE(b.has_value());
	EXPECT_FLOAT_EQ(1.f, b->getMin().x);
	EXPECT_FLOAT_EQ(3.f, b->getMax().x);
	EXPECT_FALSE(selectionCentroid(s, {}).has_value());
	EXPECT_FALSE(selectionCentroid(s, { 9 }).has_value());
}

// ---- edits ------------------------------------------------------------------------------------------------------------------

TEST(PointSetEditTest, TranslateMovesOnlyTheSelectedPointsAndLeavesTheInputAlone)
{
	const PointSet s = makeSet(4);
	const auto out = transformSelected(s, { 1, 2 }, translationMatrix({ 0.f, 5.f, 0.f }));
	ASSERT_TRUE(out.has_value());
	EXPECT_FLOAT_EQ(0.f, out->positions[0].y);
	EXPECT_FLOAT_EQ(5.f, out->positions[1].y);
	EXPECT_FLOAT_EQ(5.f, out->positions[2].y);
	EXPECT_FLOAT_EQ(0.f, out->positions[3].y);
	EXPECT_EQ(s.radii, out->radii);       // a move never resizes
	EXPECT_EQ(s.colors, out->colors);
	EXPECT_FLOAT_EQ(0.f, s.positions[1].y);   // the input is untouched
}

TEST(PointSetEditTest, RotateAboutAPivotAndNormalsFollow)
{
	PointSet s = makeSet(2, true);
	s.positions[1] = { 2.f, 0.f, 0.f };
	// A quarter turn about +Y around (1,0,0): (2,0,0) -> (1,0,-1); the normal (0,1,0) is unchanged.
	const auto out = transformSelected(s, { 1 }, rotationAbout({ 0.f, 90.f, 0.f }, { 1.f, 0.f, 0.f }));
	ASSERT_TRUE(out.has_value());
	EXPECT_NEAR(1.f, out->positions[1].x, 1.0e-5f);
	EXPECT_NEAR(-1.f, out->positions[1].z, 1.0e-5f);
	EXPECT_NEAR(1.f, out->normals[1].y, 1.0e-5f);
	// A quarter turn about +Z turns the normal (0,1,0) to (-1,0,0).
	const auto z = transformSelected(s, { 0 }, rotationAbout({ 0.f, 0.f, 90.f }, { 0.f, 0.f, 0.f }));
	ASSERT_TRUE(z.has_value());
	EXPECT_NEAR(-1.f, z->normals[0].x, 1.0e-5f);
}

TEST(PointSetEditTest, ScaleAboutThePivotAndASingularScaleIsRefusedWithNormals)
{
	const PointSet s = makeSet(3, true);
	const auto out = transformSelected(s, { 0, 2 }, scaleAbout({ 2.f, 1.f, 1.f }, { 1.f, 0.f, 0.f }));
	ASSERT_TRUE(out.has_value());
	EXPECT_FLOAT_EQ(-1.f, out->positions[0].x);   // 1 + (0 - 1) * 2
	EXPECT_FLOAT_EQ(3.f, out->positions[2].x);    // 1 + (2 - 1) * 2
	EXPECT_FLOAT_EQ(1.f, out->positions[1].x);    // not selected
	EXPECT_FALSE(transformSelected(s, { 0 }, scaleAbout({ 0.f, 1.f, 1.f }, { 0.f, 0.f, 0.f })).has_value());
}

TEST(PointSetEditTest, DeleteKeepsTheOrderOfTheRest)
{
	const PointSet s = makeSet(5, true);
	const auto out = deleteSelected(s, { 1, 3 });
	ASSERT_TRUE(out.has_value());
	ASSERT_EQ(3u, out->size());
	EXPECT_FLOAT_EQ(0.f, out->positions[0].x);
	EXPECT_FLOAT_EQ(2.f, out->positions[1].x);
	EXPECT_FLOAT_EQ(4.f, out->positions[2].x);
	EXPECT_EQ(3u, out->normals.size());
	EXPECT_TRUE(out->validate().empty());
	EXPECT_EQ(0u, deleteSelected(s, allIndices(5))->size());   // an empty cloud is allowed
}

TEST(PointSetEditTest, DuplicateAppendsCopiesAndSelectsThem)
{
	const PointSet s = makeSet(3);
	const auto out = duplicateSelected(s, { 0, 2 }, { 0.f, 0.f, 7.f });
	ASSERT_TRUE(out.has_value());
	ASSERT_EQ(5u, out->points.size());
	EXPECT_EQ((PointIndexList{ 3, 4 }), out->newSelection);
	EXPECT_FLOAT_EQ(7.f, out->points.positions[3].z);
	EXPECT_FLOAT_EQ(2.f, out->points.positions[4].x);
	EXPECT_EQ(s.radii[2], out->points.radii[4]);
	EXPECT_EQ(s.colors[0], out->points.colors[3]);
	EXPECT_TRUE(out->points.validate().empty());
}

TEST(PointSetEditTest, ExtractColorAndRadius)
{
	const PointSet s = makeSet(4);
	const auto part = extractSelected(s, { 1, 2 });
	ASSERT_TRUE(part.has_value());
	EXPECT_EQ(2u, part->size());
	EXPECT_FLOAT_EQ(1.f, part->positions[0].x);

	const auto colored = setSelectedColor(s, { 2 }, { 1.f, 0.f, 0.f });
	ASSERT_TRUE(colored.has_value());
	EXPECT_FLOAT_EQ(1.f, colored->colors[2].x);
	EXPECT_EQ(s.colors[1], colored->colors[1]);
	const auto sized = setSelectedRadius(s, { 0, 3 }, 0.5f);
	ASSERT_TRUE(sized.has_value());
	EXPECT_FLOAT_EQ(0.5f, sized->radii[3]);
	EXPECT_EQ(s.radii[1], sized->radii[1]);

	EXPECT_FALSE(setSelectedRadius(s, { 0 }, 0.f).has_value());
	EXPECT_FALSE(setSelectedRadius(s, { 0 }, std::nanf("")).has_value());
	EXPECT_FALSE(setSelectedColor(s, { 0 }, { std::nanf(""), 0.f, 0.f }).has_value());
}

TEST(PointSetEditTest, AnEmptySelectionChangesNothingAndABadIndexIsRefused)
{
	const PointSet s = makeSet(3);
	EXPECT_EQ(s.positions, transformSelected(s, {}, translationMatrix({ 1.f, 1.f, 1.f }))->positions);
	EXPECT_EQ(s.positions, deleteSelected(s, {})->positions);
	EXPECT_TRUE(duplicateSelected(s, {}, {})->newSelection.empty());
	EXPECT_FALSE(transformSelected(s, { 3 }, glm::mat4(1.f)).has_value());
	EXPECT_FALSE(deleteSelected(s, { 5 }).has_value());
	EXPECT_FALSE(deleteSelected(s, { 1, 1 }).has_value());   // not strictly increasing
	EXPECT_FALSE(extractSelected(s, { 2, 1 }).has_value());
}

// ---- picking ----------------------------------------------------------------------------------------------------------------

TEST(PointSetPickTest, ClickPicksTheNearestPointOnScreenWithinReach)
{
	const PointSet s = depthColumn();
	const auto view = frontView();
	PointPickOptions o;
	// The off-axis point (2,0,0) is at x = 100 + 2/5 * 100 = 140 px.
	const auto side = pickPoint(s, view, 141.0, 100.0, o, {});
	ASSERT_TRUE(side.has_value());
	EXPECT_EQ(3u, *side);
	EXPECT_FALSE(pickPoint(s, view, 170.0, 100.0, o, {}).has_value());   // too far from anything
	EXPECT_FALSE(pickPoint(s, view, 100.0, 20.0, o, {}).has_value());
}

TEST(PointSetPickTest, PointsBehindAnotherPointAreHiddenUnlessXRay)
{
	const PointSet s = depthColumn();
	const auto view = frontView();
	PointPickOptions o;
	const auto front = pickPoint(s, view, 100.0, 100.0, o, {});
	ASSERT_TRUE(front.has_value());
	EXPECT_EQ(0u, *front);
	// A box over the axis selects only the front point of the column; x-ray also takes the two behind it.
	EXPECT_EQ((PointIndexList{ 0 }), boxSelectPoints(s, view, 90, 90, 110, 110, o, {}));
	o.xray = true;
	EXPECT_EQ((PointIndexList{ 0, 1, 2 }), boxSelectPoints(s, view, 90, 90, 110, 110, o, {}));
}

TEST(PointSetPickTest, AMeshInFrontHidesPointsBehindItButNotThoseInFront)
{
	PointSet s = depthColumn();
	s.positions[0].z = 3.f;     // in front of the wall at z = 1
	const auto view = frontView();
	// A wall at z = 1 covering the view axis.
	const std::vector<OccluderTriangle> wall = {
		{ { -3.f, -3.f, 1.f }, { 3.f, -3.f, 1.f }, { 3.f, 3.f, 1.f } },
		{ { -3.f, -3.f, 1.f }, { 3.f, 3.f, 1.f }, { -3.f, 3.f, 1.f } } };
	PointPickOptions o;
	EXPECT_EQ((PointIndexList{ 0 }), boxSelectPoints(s, view, 90, 90, 110, 110, o, wall));
	EXPECT_TRUE(!pickPoint(s, view, 140.0, 100.0, o, wall).has_value() || *pickPoint(s, view, 140.0, 100.0, o, wall) != 3u);   // (2,0,0) is behind the wall
	o.xray = true;
	EXPECT_EQ((PointIndexList{ 0, 1, 2 }), boxSelectPoints(s, view, 90, 90, 110, 110, o, wall));
}

TEST(PointSetPickTest, RectangleCornersMayBeInAnyOrderAndPointsBehindTheCameraAreNeverPicked)
{
	PointSet s = depthColumn();
	s.positions[3] = { 0.f, 0.f, 8.f };   // behind the camera at z = 5
	const auto view = frontView();
	PointPickOptions o;
	o.xray = true;
	EXPECT_EQ(boxSelectPoints(s, view, 90, 90, 110, 110, o, {}), boxSelectPoints(s, view, 110, 110, 90, 90, o, {}));
	EXPECT_EQ((PointIndexList{ 0, 1, 2 }), boxSelectPoints(s, view, 0, 0, 200, 200, o, {}));
	EXPECT_TRUE(boxSelectPoints(s, view, 150, 150, 190, 190, o, {}).empty());
}

TEST(PointSetPickTest, TheCloudsModelMatrixIsApplied)
{
	const PointSet s = depthColumn();
	auto view = frontView();
	view.model = glm::translate(glm::mat4(1.f), glm::vec3(2.f, 0.f, 0.f));   // the whole cloud moves +2 in x
	PointPickOptions o;
	// The column of three now lies at x = 140 px.
	EXPECT_EQ((PointIndexList{ 0 }), boxSelectPoints(s, view, 130, 90, 150, 110, o, {}));
}

// ---- generation -------------------------------------------------------------------------------------------------------------

TEST(PointSetGenerateTest, VerticesBecomeOnePointEach)
{
	const auto set = pointsFromVertices({ { 0, 0, 0 }, { 1, 2, 3 } }, { 0.2f, 0.4f, 0.6f }, 0.05f);
	ASSERT_EQ(2u, set.size());
	EXPECT_FLOAT_EQ(3.f, set.positions[1].z);
	EXPECT_FLOAT_EQ(0.4f, set.colors[0].y);
	EXPECT_FLOAT_EQ(0.05f, set.radii[1]);
	EXPECT_TRUE(set.validate().empty());
}

TEST(PointSetGenerateTest, SurfaceSamplingIsDeterministicAndOnTheSurface)
{
	const std::vector<SurfaceTriangle> tris = { { { 0, 0, 0 }, { 2, 0, 0 }, { 0, 0, 2 } } };   // y = 0 plane, area 2
	const auto a = samplePointsOnSurface(tris, 500, 7, { 1, 1, 1 }, 0.01f);
	const auto b = samplePointsOnSurface(tris, 500, 7, { 1, 1, 1 }, 0.01f);
	const auto c = samplePointsOnSurface(tris, 500, 8, { 1, 1, 1 }, 0.01f);
	ASSERT_EQ(500u, a.size());
	EXPECT_EQ(a.positions, b.positions);        // the same seed gives the same points
	EXPECT_NE(a.positions, c.positions);
	EXPECT_TRUE(a.validate().empty());
	for (size_t i = 0; i < a.size(); ++i) {
		EXPECT_NEAR(0.f, a.positions[i].y, 1.0e-6f);
		EXPECT_GE(a.positions[i].x, -1.0e-6f);
		EXPECT_GE(a.positions[i].z, -1.0e-6f);
		EXPECT_LE(a.positions[i].x + a.positions[i].z, 2.0f + 1.0e-5f);   // inside the triangle
		EXPECT_NEAR(1.f, glm::length(a.normals[i]), 1.0e-5f);
	}
}

TEST(PointSetGenerateTest, PointsFollowAreaAndDegenerateTrianglesGetNone)
{
	// Areas 1 and 3: about a quarter / three quarters of the points. A third triangle has no area.
	const std::vector<SurfaceTriangle> tris = {
		{ { 0, 0, 0 }, { 2, 0, 0 }, { 0, 1, 0 } },                     // area 1, z = 0
		{ { 0, 0, 5 }, { 6, 0, 5 }, { 0, 1, 5 } },                     // area 3, z = 5
		{ { 9, 9, 9 }, { 9, 9, 9 }, { 9, 9, 9 } } };                   // none
	EXPECT_NEAR(4.0, totalArea(tris), 1.0e-9);
	const auto set = samplePointsOnSurface(tris, 20000, 1, { 1, 1, 1 }, 0.01f);
	size_t onSmall = 0, onDegenerate = 0;
	for (const auto& p : set.positions) {
		if (std::abs(p.z) < 1.0e-4f) ++onSmall;
		if (p.x > 8.f) ++onDegenerate;
	}
	EXPECT_EQ(0u, onDegenerate);
	EXPECT_NEAR(0.25, static_cast<double>(onSmall) / 20000.0, 0.02);
}

TEST(PointSetGenerateTest, NothingToSampleGivesAnEmptySet)
{
	EXPECT_TRUE(samplePointsOnSurface({}, 10, 1, { 1, 1, 1 }, 0.1f).empty());
	EXPECT_TRUE(samplePointsOnSurface({ { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 } } }, 10, 1, { 1, 1, 1 }, 0.1f).empty());   // collinear
	EXPECT_TRUE(samplePointsOnSurface({ { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } } }, 0, 1, { 1, 1, 1 }, 0.1f).empty());
}
