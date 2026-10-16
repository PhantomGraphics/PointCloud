#pragma once

#include "PLYFile.h"
#include <filesystem>

namespace Phantom {
	namespace PC {

		/// @brief Writes point cloud data from a PLYFile to a PLY file.
		/// Supports ASCII and binary little-endian output formats.
		class PLYFileWriter
		{
		public:
			/// @brief Writes the point cloud in ASCII format.
			/// @param filename Output file path.
			/// @param file The point cloud data to write.
			/// @return true on success, false on failure.
			bool writeAscii(const std::filesystem::path& filename, const PLYFile& file);

			/// @brief Writes the point cloud in binary little-endian format.
			/// @param filename Output file path.
			/// @param file The point cloud data to write.
			/// @return true on success, false on failure.
			bool writeBinary(const std::filesystem::path& filename, const PLYFile& file);
		};

	}
}
