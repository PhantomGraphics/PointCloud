#include "RadiusOutlierFilter.h"

namespace Phantom {
	namespace PC {

		void RadiusOutlierFilter::execute(const float radius, const int minNeighbors)
		{
			inlierIndices.clear();
			outlierIndices.clear();

			const size_t n = pointCloud.size();
			if (n == 0) {
				return;
			}

			const float r2 = radius * radius;

			// O(n^2) brute force: the outer loop is fully independent per point, so it's the one
			// parallelized. Classification is written to a mask (see SORFilter::execute for why a
			// plain std::vector<bool>/push_back isn't safe here) and split single-threaded below,
			// preserving the original ascending-index order in inlierIndices/outlierIndices.
			std::vector<char> isInlier(n);
			const int nInt = static_cast<int>(n);
#pragma omp parallel for
			for (int ii = 0; ii < nInt; ++ii) {
				const size_t i = static_cast<size_t>(ii);
				int count = 0;
				for (size_t j = 0; j < n; ++j) {
					if (j == i) continue;
					const float dx = pointCloud[i].x - pointCloud[j].x;
					const float dy = pointCloud[i].y - pointCloud[j].y;
					const float dz = pointCloud[i].z - pointCloud[j].z;
					if (dx * dx + dy * dy + dz * dz <= r2) ++count;
				}
				isInlier[i] = (count >= minNeighbors) ? 1 : 0;
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
