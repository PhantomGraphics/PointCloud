#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Statistical Outlier Removal (SOR) filter.
		/// For each point, computes the mean distance to its k nearest neighbors, then classifies
		/// points whose mean distance exceeds (global mean + mulThreshold * global stddev) as outliers.
		class SORFilter
		{
		public:
			SORFilter() = default;

			~SORFilter() = default;

			/// @brief Adds a point to be filtered.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Executes the SOR filter.
			/// @param k Number of nearest neighbors used to compute each point's mean distance.
			///          Internally clamped to (point count - 1).
			/// @param mulThreshold Standard deviation multiplier used to derive the outlier threshold.
			void execute(const int k, const float mulThreshold);

			/// @brief Returns the indices (into the added points, ascending order) classified as inliers.
			std::vector<int> getInlierIndices() const { return inlierIndices; }

			/// @brief Returns the indices (into the added points, ascending order) classified as outliers.
			std::vector<int> getOutlierIndices() const { return outlierIndices; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<int> inlierIndices;
			std::vector<int> outlierIndices;
		};

	}
}
