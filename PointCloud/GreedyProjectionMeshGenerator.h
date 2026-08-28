#pragma once

#include <vector>
#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Triangle3d.h"

namespace Phantom
{
	namespace PC
	{
		/// @brief Generates a triangle mesh from a point cloud using the Greedy Projection algorithm.
		/// Projects points onto a local plane and connects neighboring points to form triangles.
		class GreedyProjectionMeshGenerator
		{
		public:
			/// @brief Generates a mesh from the given point cloud.
			/// @param points Input point cloud as a list of 3D coordinates.
			void generate(const std::vector<Math::Vector3df>& points);

			/// @brief Returns generated triangles.
			const std::vector<Math::Triangle3df>& getTriangles() const { return triangles; }

		private:
			std::vector<Math::Triangle3df> triangles;
		};
	}
}
