#include "SORFilter.h"

#include "../../CGLib/Space/Space/KDTree.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Phantom {
	namespace PC {

		void SORFilter::execute(const int k, const float mulThreshold)
		{
			inlierIndices.clear();
			outlierIndices.clear();

			const size_t n = pointCloud.size();
			if (n == 0) {
				return;
			}

			const size_t kk = (k > 0) ? std::min<size_t>(static_cast<size_t>(k), n - 1) : 0;
			if (kk == 0) {
				// Not enough neighbors to evaluate distances against; treat every point as an inlier.
				inlierIndices.resize(n);
				std::iota(inlierIndices.begin(), inlierIndices.end(), 0);
				return;
			}

			Phantom::Space::KDTree tree;
			tree.build(pointCloud);

			std::vector<float> meanDists(n, 0.0f);
			const int nInt = static_cast<int>(n);
#pragma omp parallel for
			for (int ii = 0; ii < nInt; ++ii) {
				const size_t i = static_cast<size_t>(ii);
				// +1 since the query point itself is typically returned as its own nearest neighbor;
				// skip it below rather than assuming it's always first (ties at distance 0 aren't
				// guaranteed to put the query point ahead of an exact-duplicate neighbor).
				const auto indices = tree.findKNearestIndices(pointCloud[i], kk + 1);
				float sum = 0.0f;
				size_t count = 0;
				for (const int idx : indices) {
					if (static_cast<size_t>(idx) == i) continue;
					const auto d = pointCloud[i] - pointCloud[static_cast<size_t>(idx)];
					sum += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
					if (++count == kk) break;
				}
				meanDists[i] = (count > 0) ? sum / static_cast<float>(count) : 0.0f;
			}

			double globalMean = 0.0;
			for (float d : meanDists) globalMean += static_cast<double>(d);
			globalMean /= static_cast<double>(n);

			double globalVar = 0.0;
			for (float d : meanDists) {
				const double diff = static_cast<double>(d) - globalMean;
				globalVar += diff * diff;
			}
			globalVar /= static_cast<double>(n);
			const double threshold = globalMean + static_cast<double>(mulThreshold) * std::sqrt(globalVar);

			// Classify in parallel into a mask first: std::vector<bool>'s bit-packed storage would
			// make concurrent writes to adjacent indices race, and push_back on a shared vector
			// isn't thread-safe either, so each point's verdict is written to its own byte here and
			// the (cheap, O(n)) split into inlierIndices/outlierIndices happens single-threaded
			// below, preserving the original ascending-index order.
			std::vector<char> isInlier(n);
#pragma omp parallel for
			for (int ii = 0; ii < nInt; ++ii) {
				isInlier[static_cast<size_t>(ii)] = (static_cast<double>(meanDists[static_cast<size_t>(ii)]) <= threshold) ? 1 : 0;
			}

			for (size_t i = 0; i < n; ++i) {
				if (isInlier[i]) {
					inlierIndices.push_back(static_cast<int>(i));
				} else {
					outlierIndices.push_back(static_cast<int>(i));
				}
			}
		}

	}
}
