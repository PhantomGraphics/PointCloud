#pragma once

#include "../../CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Stores point cloud data loaded from a PLY file.
		/// Positions are always present; colors (RGB normalized to [0,1]) and normals are
		/// filled only when the source file declares those vertex properties, so callers
		/// must check hasColors()/hasNormals() before indexing them.
		class PLYFile
		{
		public:
			PLYFile() = default;
			~PLYFile() = default;

			/// @brief Removes all stored points, colors and normals.
			void clear() { points.clear(); colors.clear(); normals.clear(); }

			/// @brief Returns a const reference to the point list.
			/// @return Const reference to the vector of 3D coordinates.
			const std::vector<Phantom::Math::Vector3df>& getPoints() const { return points; }

			/// @brief Returns a reference to the point list.
			/// @return Reference to the vector of 3D coordinates.
			std::vector<Phantom::Math::Vector3df>& getPoints() { return points; }

			/// @brief Returns a const reference to the per-point RGB colors, normalized to [0,1].
			/// @return Const reference to the color list (empty when the file had no color properties).
			const std::vector<Phantom::Math::Vector3df>& getColors() const { return colors; }

			/// @brief Returns a reference to the per-point RGB colors, normalized to [0,1].
			std::vector<Phantom::Math::Vector3df>& getColors() { return colors; }

			/// @brief Returns true when per-point colors were read from the file.
			bool hasColors() const { return !colors.empty(); }

			/// @brief Returns a const reference to the per-point normals.
			/// @return Const reference to the normal list (empty when the file had no normal properties).
			const std::vector<Phantom::Math::Vector3df>& getNormals() const { return normals; }

			/// @brief Returns a reference to the per-point normals.
			std::vector<Phantom::Math::Vector3df>& getNormals() { return normals; }

			/// @brief Returns true when per-point normals were read from the file.
			bool hasNormals() const { return !normals.empty(); }

			/// @brief Returns the number of points.
			/// @return The size of the point cloud.
			size_t size() const { return points.size(); }

		private:
			std::vector<Phantom::Math::Vector3df> points;
			std::vector<Phantom::Math::Vector3df> colors;
			std::vector<Phantom::Math::Vector3df> normals;
		};

	}
}
