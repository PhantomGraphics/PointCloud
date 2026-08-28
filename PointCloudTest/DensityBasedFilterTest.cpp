#include "pch.h"

#include "../PointCloud/DensityBasedFilter.h"
#include "../../CGLib/Math/Vector3d.h"

#include <algorithm>

using namespace Phantom::Math;
using namespace Phantom::PC;

TEST(DensityBasedFilterTest, SeparatesIsolatedOutlier)
{
	DensityBasedFilter filter;
	filter.add(Vector3df(0.00f, 0.00f, 0.00f));
	filter.add(Vector3df(0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(-0.05f, 0.00f, 0.00f));
	filter.add(Vector3df(0.00f, 0.05f, 0.00f));
	filter.add(Vector3df(0.00f, -0.05f, 0.00f));
	filter.add(Vector3df(10.0f, 10.0f, 10.0f));

	filter.execute(0.3);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	ASSERT_EQ(1u, outliers.size());
	EXPECT_EQ(5, outliers[0]);
}

TEST(DensityBasedFilterTest, UniformDensityClassifiesAllPointsAsInliers)
{
	DensityBasedFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(-1.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.0f, 1.0f, 0.0f));
	filter.add(Vector3df(0.0f, -1.0f, 0.0f));

	filter.execute(1.5);

	const auto inliers = filter.getInlierIndices();
	const auto outliers = filter.getOutlierIndices();
	EXPECT_EQ(5u, inliers.size());
	EXPECT_TRUE(outliers.empty());
}

TEST(DensityBasedFilterTest, ExecuteDoesNotAccumulateIndicesAcrossCalls)
{
	DensityBasedFilter filter;
	filter.add(Vector3df(0.0f, 0.0f, 0.0f));
	filter.add(Vector3df(0.1f, 0.0f, 0.0f));
	filter.add(Vector3df(5.0f, 5.0f, 5.0f));

	filter.execute(0.3);
	const auto firstTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	ASSERT_EQ(3u, firstTotal);

	filter.execute(0.3);
	const auto secondTotal = filter.getInlierIndices().size() + filter.getOutlierIndices().size();
	EXPECT_EQ(3u, secondTotal);
}
