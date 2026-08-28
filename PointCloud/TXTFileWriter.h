#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <string>
#include <filesystem>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Writes point cloud data to a text file.
		/// Each point is output as a single "x y z" line.
		class TXTFileWriter
		{
		public:
			/// @brief Adds a point to be written.
			/// @param position The 3D coordinate to add.
			void add(const Math::Vector3df& position) { positions.push_back(position); }

			/// @brief Writes all added points to the specified file path.
			/// @param filename Output file path.
			/// @return true on success, false on failure.
			bool write(const std::filesystem::path& filename);

			/// @brief Writes all added points to the given output stream.
			/// @param stream The output stream to write to.
			/// @return true on success, false on failure.
			bool write(std::ostream& stream);

		private:
			std::vector<Math::Vector3df> positions;
		};

	}
}
