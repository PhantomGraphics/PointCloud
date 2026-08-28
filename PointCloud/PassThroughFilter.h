#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Axis to evaluate in PassThroughFilter.
		enum class PassThroughAxis {
			X,
			Y,
			Z,
		};

		/// @brief Keeps only points whose coordinate on the given axis falls within [minValue, maxValue].
		class PassThroughFilter
		{
		public:
			PassThroughFilter() = default;

			~PassThroughFilter() = default;

			/// @brief Adds a point to be filtered.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Executes the pass-through filter.
			/// @param axis Axis whose coordinate is evaluated against [minValue, maxValue].
			/// @param minValue Inclusive lower bound.
			/// @param maxValue Inclusive upper bound.
			void execute(const PassThroughAxis axis, const float minValue, const float maxValue);

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
