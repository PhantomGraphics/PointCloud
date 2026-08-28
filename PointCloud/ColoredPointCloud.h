#pragma once

#include "IPointCloud.h"
#include "PointCloudFileLoader.h"
#include <utility>
#include <vector>

namespace Phantom
{
	namespace PC
	{
		/// @brief IPointCloud-compliant wrapper around PointCloudColoredData.
		///
		/// PointCloudColoredData (see PointCloudFileLoader.h) is the SoA container already
		/// used by loadPointCloud()/savePointCloud() and most algorithms (see
		/// docs/todo/PLAN_pointcloud_soa_migration.md Phase 1). ColoredPointCloud wraps one so
		/// it can also be passed through APIs that accept `const IPointCloud&`, while still
		/// keeping direct access to colors/normals/scalars via getData()/getColor()/etc.
		class ColoredPointCloud : public IPointCloud
		{
		public:
			ColoredPointCloud() = default;

			/// @brief Wraps an existing PointCloudColoredData (e.g. the output of
			/// loadPointCloud()) so it can be used through the IPointCloud interface.
			explicit ColoredPointCloud(PointCloudColoredData data) :
				data(std::move(data))
			{}

			size_t size() const override { return data.size(); }

			Math::Vector3df getPosition(size_t index) const override { return data.positions[index]; }

			const std::vector<Math::Vector3df>& getPositions() const override { return data.positions; }

			void setPositions(std::vector<Math::Vector3df> positions) override { data.positions = std::move(positions); }

			void addPosition(const Math::Vector3df& position) override { data.positions.push_back(position); }

			void addColor(const Math::Vector3df& color) { data.colors.push_back(color); }
			void addNormal(const Math::Vector3df& normal) { data.normals.push_back(normal); }
			void addScalar(float value) { data.scalars.push_back(value); }

			Math::Vector3df getColor(size_t index) const { return data.colors[index]; }
			Math::Vector3df getNormal(size_t index) const { return data.normals[index]; }
			float getScalar(size_t index) const { return data.scalars[index]; }

			bool hasColors() const { return data.hasColors(); }
			bool hasNormals() const { return data.hasNormals(); }
			bool hasScalars() const { return data.hasScalars(); }

			/// @brief Returns a const reference to the underlying PointCloudColoredData
			/// (e.g. to pass to savePointCloud()).
			const PointCloudColoredData& getData() const { return data; }
			/// @brief Returns a mutable reference to the underlying PointCloudColoredData.
			PointCloudColoredData& getData() { return data; }

		private:
			PointCloudColoredData data;
		};
	}
}
