#pragma once

#include "PointSet.h"

#include <filesystem>
#include <string>

namespace Phantom {
	namespace PC {

		/// @brief What a PLY import found in the file and what it filled in.
		struct PointSetImportInfo
		{
			bool hadColor = false;    ///< false: every point got the default colour
			bool hadRadius = false;   ///< false: every point got `radius` (see PointSetImportOptions)
			bool hadNormals = false;
			float appliedRadius = 0.0f; ///< the radius assigned when the file had none (0 otherwise)
		};

		struct PointSetImportOptions
		{
			/// Linear RGB for a file without colours.
			Math::Vector3df defaultColor = Math::Vector3df(0.8f, 0.8f, 0.8f);
			/// Radius for a file without radii. 0 = derive it from the bounds (defaultRadiusForBounds).
			float defaultRadius = 0.0f;
		};

		/// @brief Reads an ASCII or binary PLY into a PointSet. PLY colours are sRGB and are converted
		/// to linear; a PLY without colours/radii gets the defaults in `options`.
		/// A file with a non-finite coordinate, colour or normal, or a non-finite / non-positive
		/// radius, is rejected whole (nothing is returned). An empty vertex list is valid.
		/// @return false and a diagnostic in `error` on failure; `out` is left unchanged then.
		bool loadPointSetFromPly(const std::filesystem::path& path, const PointSetImportOptions& options,
			PointSet& out, PointSetImportInfo& info, std::string& error);

		/// @brief Writes a PointSet as PLY (x y z [nx ny nz] red green blue radius). Colours are
		/// converted from linear to sRGB 8-bit.
		bool savePointSetToPly(const std::filesystem::path& path, const PointSet& points, bool binary, std::string& error);

		/// Version of the binary asset format written by savePointSetBinary.
		constexpr uint32_t kPointSetBinaryVersion = 1;

		/// @brief Writes the lossless binary asset: magic, version, count, content hash, then the
		/// float arrays (positions, linear colours, radii, normals when present). The file is written
		/// to a temporary sibling and renamed, so a failure never leaves a truncated asset.
		bool savePointSetBinary(const std::filesystem::path& path, const PointSet& points, std::string& error);

		/// @brief Reads a binary asset written by savePointSetBinary. Verifies the magic, a version this
		/// build understands, the exact file size for the declared count (before allocating) and the
		/// content hash, then the PointSet invariants.
		/// @param[out] hash The content hash recorded in the file.
		bool loadPointSetBinary(const std::filesystem::path& path, PointSet& out, uint64_t& hash, std::string& error);

		/// Lowercase 16-digit hex of a content hash (used in asset file names).
		std::string hashToHex(uint64_t hash);

	}
}
