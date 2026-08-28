#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Classifies point cloud points into inliers and outliers based on local density.
		/// Points with fewer neighbors than expected within the search radius are classified as outliers.
		class DensityBasedFilter
		{
		public:
			DensityBasedFilter() = default;

			~DensityBasedFilter() = default;

			/// @brief Adds a point to be filtered.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Executes the density-based filtering.
			/// @param searchRadius The neighborhood search radius used to compute density.
			void execute(const double searchRadius);

			/// @brief Returns the indices of inlier points.
			/// @return A vector of indices into the added points that are classified as inliers.
			std::vector<int> getInlierIndices() const { return inlierIndices; }

			/// @brief Returns the indices of outlier points.
			/// @return A vector of indices into the added points that are classified as outliers.
			std::vector<int> getOutlierIndices() const { return outlierIndices; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<int> inlierIndices;
			std::vector<int> outlierIndices;
		};

	}
}
