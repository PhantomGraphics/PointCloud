#pragma once
#include "../../CGLib/Math/Vector3d.h"
#include <string>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Stores point cloud data loaded from a PCD file.
		/// Currently manages position data only (Vector3df).
		class PCDFile
		{
		public:
			PCDFile() = default;
			~PCDFile() = default;

			/// @brief Removes all stored points.
			void clear() { points.clear(); }

			/// @brief Returns a const reference to the point list.
			/// @return Const reference to the vector of 3D coordinates.
			const std::vector<Phantom::Math::Vector3df>& getPoints() const { return points; }

			/// @brief Returns a reference to the point list.
			/// @return Reference to the vector of 3D coordinates.
			std::vector<Phantom::Math::Vector3df>& getPoints() { return points; }

			/// @brief Returns the number of points.
			/// @return The size of the point cloud.
			size_t size() const { return points.size(); }

		private:
			std::vector<Phantom::Math::Vector3df> points;
		};
	}
}
