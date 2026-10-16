#include "PassThroughFilter.h"

namespace Phantom {
	namespace PC {

		void PassThroughFilter::execute(const PassThroughAxis axis, const float minValue, const float maxValue)
		{
			inlierIndices.clear();
			outlierIndices.clear();

			const size_t n = pointCloud.size();
			for (size_t i = 0; i < n; ++i) {
				const auto& p = pointCloud[i];
				const float v = (axis == PassThroughAxis::X) ? p.x : (axis == PassThroughAxis::Y) ? p.y : p.z;
				if (v >= minValue && v <= maxValue) {
					inlierIndices.push_back(static_cast<int>(i));
				} else {
					outlierIndices.push_back(static_cast<int>(i));
				}
			}
		}

	}
}
