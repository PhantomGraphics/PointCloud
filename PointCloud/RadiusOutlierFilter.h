#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Radius Outlier (RO) removal filter.
		/// Classifies points with fewer than minNeighbors other points within the given radius
		/// as outliers.
		class RadiusOutlierFilter
		{
		public:
			RadiusOutlierFilter() = default;

			~RadiusOutlierFilter() = default;

			/// @brief Adds a point to be filtered.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Executes the radius outlier filter.
			/// @param radius Neighborhood search radius.
			/// @param minNeighbors Minimum number of neighbors (excluding the point itself) within
			///                     radius required to classify a point as an inlier.
			void execute(const float radius, const int minNeighbors);

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
