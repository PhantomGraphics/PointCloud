#pragma once

#include "PLYFile.h"
#include <cstdint>
#include <filesystem>
#include <istream>
#include <string>
#include <vector>

namespace Phantom {
	namespace PC {

		namespace detail {
			// Scalar property types supported in PLY
			enum class PLYType {
				FLOAT32, FLOAT64,
				INT8, UINT8, INT16, UINT16, INT32, UINT32,
				UNKNOWN
			};

			/// @brief Property indices of the vertex element fields this reader understands.
			/// Every member is an index into the vertex element's scalar property list, or
			/// npos when the file does not declare that property. x/y/z are mandatory;
			/// the color and normal triplets are each used only when all three are present.
			struct PLYVertexFields {
				static constexpr size_t npos = static_cast<size_t>(-1);

				size_t x = npos, y = npos, z = npos;
				size_t r = npos, g = npos, b = npos;
				size_t nx = npos, ny = npos, nz = npos;

				bool hasPosition() const { return x != npos && y != npos && z != npos; }
				bool hasColor()    const { return r != npos && g != npos && b != npos; }
				bool hasNormal()   const { return nx != npos && ny != npos && nz != npos; }
			};
		}

		/// @brief Reads a PLY file and stores the parsed vertex data into a PLYFile object.
		/// Supports ASCII, binary little-endian, and binary big-endian formats.
		/// Vertex x/y/z positions are mandatory; red/green/blue and nx/ny/nz are read as
		/// colors and normals when present (integer color channels are normalized by their
		/// type's maximum, so uchar 0-255 becomes 0-1). Face and other elements are skipped.
		class PLYFileReader
		{
		public:
			/// @brief Reads the specified PLY file.
			/// @param filename Path to the PLY file.
			/// @return true on success, false on failure.
			bool read(const std::filesystem::path& filename);

			/// @brief Returns last error message from read operation.
			const std::string& getLastError() const { return lastError_; }

			/// @brief Returns a const reference to the parsed PLYFile.
			/// @return Const reference to the PLYFile.
			const PLYFile& getFile() const { return file; }

			/// @brief Returns a reference to the parsed PLYFile.
			/// @return Reference to the PLYFile.
			PLYFile& getFile() { return file; }

		private:
			bool readASCII(std::istream& stream, const detail::PLYVertexFields& fields,
				const std::vector<detail::PLYType>& propTypes,
				size_t propCount, size_t vertexCount);

			bool readBinary(std::istream& stream,
				const std::vector<size_t>& propSizes,
				const std::vector<detail::PLYType>& propTypes,
				const detail::PLYVertexFields& fields,
				size_t vertexCount, bool swapBytes);

			PLYFile file;
          std::string lastError_;
		};

	}
}
