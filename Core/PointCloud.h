#pragma once

#include "IPointCloud.h"
#include <utility>
#include <vector>

namespace Phantom
{
	namespace PC
	{
		/// @brief Minimal position-only point cloud (SoA: a single parallel array).
		///
		/// The lightweight sibling of ColoredPointCloud - use this when only positions are
		/// needed (e.g. RANSAC/clustering/generator output) and ColoredPointCloud (or
		/// PointCloudColoredData directly) when color/normal/scalar attributes are needed
		/// too. Both implement IPointCloud so generic code can treat them interchangeably.
		class PointCloud : public IPointCloud
		{
		public:
			PointCloud() = default;

			explicit PointCloud(std::vector<Math::Vector3df> positions) :
				positions(std::move(positions))
			{}

			void addPosition(const Math::Vector3df& position) override { positions.push_back(position); }

			size_t size() const override { return positions.size(); }

			Math::Vector3df getPosition(size_t index) const override { return positions[index]; }

			const std::vector<Math::Vector3df>& getPositions() const override { return positions; }

			void setPositions(std::vector<Math::Vector3df> newPositions) override { positions = std::move(newPositions); }

		private:
			std::vector<Math::Vector3df> positions;
		};
	}
}
