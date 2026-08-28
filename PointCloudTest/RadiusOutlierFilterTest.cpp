#include "pch.h"

#include "../PointCloud/RadiusOutlierFilter.h"
#include "../../CGLib/Math/Vector3d.h"

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(RadiusOutlierFilterTest, SeparatesIsolatedOutlier)
{
	RadiusOutlierFilter filter;
	filter.add(Vector3df(0.00f, 0.00f, 0.00f));
	filter.add(Vector3df(0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(-0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(0.00f, 0.05f, 0.00f));
	filter.add(Vector3df(0.00f, -0.05f, 0.00f));
	filter.add(Vector3df(10.0f, 10.0f, 10.0f));

	filter.execute(0.3f, 2);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	ASSERT_EQ(1u, outliers.size());
	EXPECT_EQ(5, outliers[0]);
}

TEST(RadiusOutlierFilterTest, UniformDensityClassifiesAllPointsAsInliers)
{
	RadiusOutlierFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(-1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.0f, 1.0f, 0.0f));
	filter.add(Vector3df(0.0f, -1.0f, 0.0f));

	filter.execute(1.5f, 2);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	EXPECT_TRUE(outliers.empty());
}

TEST(RadiusOutlierFilterTest, HighMinNeighborsClassifiesAllPointsAsOutliers)
{
	RadiusOutlierFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(-1.0f, 0.0f, 0.0f));

	// Each point has at most 2 neighbors within radius; requiring 5 makes every point an outlier.
	filter.execute(2.5f, 5);

	EXPECT_TRUE(filter.getInlierIndices().empty());
	EXPECT_EQ(3u, filter.getOutlierIndices().size());
}

TEST(RadiusOutlierFilterTest, ExecuteDoesNotAccumulateIndicesAcrossCalls)
{
	RadiusOutlierFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.1f, 0.0f, 0.0f));
	filter.add(Vector3df(5.0f, 5.0f, 5.0f));

	filter.execute(0.3f, 1);
	const auto firstTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	ASSERT_EQ(3u, firstTotal);

	filter.execute(0.3f, 1);
	const auto secondTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	EXPECT_EQ(3u, secondTotal);
}

TEST(RadiusOutlierFilterTest, EmptyInputProducesNoIndices)
{
	RadiusOutlierFilter filter;
	filter.execute(1.0f, 1);
	EXPECT_TRUE(filter.getInlierIndices().empty());
	EXPECT_TRUE(filter.getOutlierIndices().empty());
}
