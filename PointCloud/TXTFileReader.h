#pragma once

#include <string>
#include <filesystem>
#include <vector>

#include "../../CGLib/Math/Vector3d.h"

namespace Phantom {
	namespace PC {

		/// @brief Reads a text-format point cloud file.
		/// Parses each line as "x y z" whitespace-separated coordinates.
		class TXTFileReader
		{
		public:
			/// @brief Reads the point cloud from the given file path.
			/// @param filename Path to the text file.
			/// @return true on success, false on failure.
			bool read(const std::filesystem::path& filename);

			/// @brief Reads the point cloud from an input stream.
			/// @param stream The input stream to read from.
			/// @return true on success, false on failure.
			bool read(std::istream& stream);

			/// @brief Returns the list of parsed 3D positions.
			/// @return A vector of Vector3df positions.
			std::vector<Math::Vector3df> getPositions() const { return positions; }

			/// @brief Returns last error message from read operation.
			const std::string& getLastError() const { return lastError_; }

		private:
			std::vector<Math::Vector3df> positions;
          std::string lastError_;
		};

	}
}
