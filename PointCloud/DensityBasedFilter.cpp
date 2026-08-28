#include "DensityBasedFilter.h"

#include "DensityEstimator.h"

#include "../../CGLib/Math/Statistics.h"

#include <limits>

using namespace Phantom::Math;
using namespace Phantom::PC;

void DensityBasedFilter::execute(const double searchRadius)
{
	inlierIndices.clear();
	outlierIndices.clear();

	DensityEstimator estimator;
	for (const auto& point : pointCloud) {
		estimator.add(point);
	}
	estimator.estimate(searchRadius);
	const auto densities = estimator.getDensities();

	Statistics<double> stat(densities);
	const auto average = stat.getAverage();
	const auto sd = stat.getStandardDeviation();
	const auto threshold = (sd > 0.0) ? 2.0 * sd : std::numeric_limits<double>::max();

	// See SORFilter::execute for why classification goes through a mask (char, not
	// vector<bool>) instead of push_back'ing directly from the parallel loop.
	const int n = static_cast<int>(pointCloud.size());
	std::vector<char> isInlier(static_cast<size_t>(n));
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		const auto d = densities[i];
		isInlier[static_cast<size_t>(i)] = (::fabs(d - average) <= threshold) ? 1 : 0;
	}

	for (int i = 0; i < n; ++i) {
		if (isInlier[static_cast<size_t>(i)]) {
			inlierIndices.push_back(i);
		}
		else {
			outlierIndices.push_back(i);
		}
	}
}
