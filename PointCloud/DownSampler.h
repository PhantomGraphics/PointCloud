#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Downsamples a point cloud using a voxel grid.
		/// Each point is bucketed into a fixed-size grid cell by its own coordinates
		/// (floor(position / voxelSize) per axis, independent of point order), and every
		/// occupied cell is replaced by the centroid of the points inside it. This matches
		/// the standard "voxel grid downsampling" used by e.g. PCL's VoxelGrid filter and
		/// Open3D's voxel_down_sample().
		class DownSampler
		{
		public:
			DownSampler() = default;

			~DownSampler() = default;

			/// @brief Adds a point to be downsampled.
			/// @param position The 3D coordinate of the point.
			void add(const Math::Vector3df& position) { this->pointCloud.push_back(position); }

			/// @brief Executes the downsampling.
			/// @param voxelSize The voxel grid cell size (edge length).
			void execute(const double voxelSize);

			/// @brief Returns the downsampled point cloud.
			/// @return A vector of representative 3D coordinates after downsampling.
			std::vector<Math::Vector3df> getDownSampled() const { return downSampled; }

		private:
			std::vector<Math::Vector3df> pointCloud;
			std::vector<Math::Vector3df> downSampled;
		};

	}
}
