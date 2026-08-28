#include "pch.h"

#include "../PointCloud/SORFilter.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(SORFilterTest, SeparatesIsolatedOutlier)
{
	SORFilter filter;
	filter.add(Vector3df(0.00f, 0.00f, 0.00f));
	filter.add(Vector3df(0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(-0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(0.00f, 0.05f, 0.00f));
	filter.add(Vector3df(0.00f, -0.05f, 0.00f));
	filter.add(Vector3df(10.0f, 10.0f, 10.0f));

	filter.execute(2, 1.0f);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	ASSERT_EQ(1u, outliers.size());
	EXPECT_EQ(5, outliers[0]);
}

TEST(SORFilterTest, UniformDensityClassifiesAllPointsAsInliers)
{
	SORFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(-1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.0f, 1.0f, 0.0f));
	filter.add(Vector3df(0.0f, -1.0f, 0.0f));

	filter.execute(2, 2.0f);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	EXPECT_TRUE(outliers.empty());
}

TEST(SORFilterTest, NonPositiveKTreatsEveryPointAsInlier)
{
	SORFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(50.0f, 50.0f, 50.0f));

	filter.execute(0, 1.0f);

	EXPECT_EQ(3u, filter.getInlierIndices().size());
	EXPECT_TRUE(filter.getOutlierIndices().empty());
}

TEST(SORFilterTest, ExecuteDoesNotAccumulateIndicesAcrossCalls)
{
	SORFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.1f, 0.0f, 0.0f));
	filter.add(Vector3df(0.2f, 0.0f, 0.0f));
	filter.add(Vector3df(5.0f, 5.0f, 5.0f));

	filter.execute(2, 1.0f);
	const auto firstTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	ASSERT_EQ(4u, firstTotal);

	filter.execute(2, 1.0f);
	const auto secondTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	EXPECT_EQ(4u, secondTotal);
}

TEST(SORFilterTest, EmptyInputProducesNoIndices)
{
	SORFilter filter;
	filter.execute(2, 1.0f);
	EXPECT_TRUE(filter.getInlierIndices().empty());
	EXPECT_TRUE(filter.getOutlierIndices().empty());
}
