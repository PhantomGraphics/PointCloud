#pragma once

#include "PCDFile.h"

#include "../../CGLib/Math/Vector3d.h"
#include <string>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Reads a PCD file and stores the parsed point cloud into a PCDFile object.
		/// Supports both ASCII and Binary PCD formats.
		class PCDFileReader
		{
		public:
			/// @brief Reads the specified PCD file.
			/// @param filename Path to the PCD file.
			/// @return true on success, false on failure.
			bool read(const std::string& filename);

			/// @brief Returns last error message from read operation.
			const std::string& getLastError() const { return lastError_; }

			/// @brief Returns a const reference to the parsed PCDFile.
			/// @return Const reference to the PCDFile.
			const PCDFile& getFile() const { return file; }

			/// @brief Returns a reference to the parsed PCDFile.
			/// @return Reference to the PCDFile.
			PCDFile& getFile() { return file; }

		private:
			/// @brief Parses an ASCII-format PCD data section.
			/// @param stream Input stream positioned at the DATA section.
			/// @param fields List of field names from the PCD header.
			/// @param pointCount Expected number of points.
			/// @return true on success.
			bool readASCII(std::istream& stream, const std::vector<std::string>& fields, size_t pointCount);

			/// @brief Parses a Binary-format PCD data section.
			/// @param stream Input stream positioned at the DATA section.
			/// @param fields List of field names from the PCD header.
			/// @param sizes Byte size of each field.
			/// @param types Type identifier of each field.
			/// @param counts Element count of each field.
			/// @param pointCount Expected number of points.
			/// @return true on success.
			bool readBinary(std::istream& stream, const std::vector<std::string>& fields, const std::vector<int>& sizes, const std::vector<char>& types, const std::vector<int>& counts, size_t pointCount);

			PCDFile file;
          std::string lastError_;
		};
	}
}
