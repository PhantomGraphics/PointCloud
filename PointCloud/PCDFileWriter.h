#pragma once

#include "PCDFile.h"

#include "../../CGLib/Math/Vector3d.h"
#include <string>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Writes point cloud data from a PCDFile to a PCD file.
		/// Supports both ASCII and Binary output formats.
		class PCDFileWriter
		{
		public:
			/// @brief Writes the point cloud in ASCII format.
			/// @param filename Output file path.
			/// @param file The point cloud data to write.
			/// @return true on success, false on failure.
			bool writeAscii(const std::string& filename, const PCDFile& file);

			/// @brief Writes the point cloud in Binary format.
			/// @param filename Output file path.
			/// @param file The point cloud data to write.
			/// @return true on success, false on failure.
			bool writeBinary(const std::string& filename, const PCDFile& file);
		};
	}
}
