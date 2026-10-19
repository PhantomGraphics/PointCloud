#include "PointSetFile.h"

#include "PLYFile.h"
#include "PLYFileReader.h"
#include "PLYFileWriter.h"

#include <bit>
#include <cstdio>
#include <cstring>
#include <fstream>

using namespace Phantom::PC;
using namespace Phantom::Math;

static_assert(std::endian::native == std::endian::little, "the binary point-set asset is little-endian");
static_assert(sizeof(Vector3df) == 12, "Vector3df must be three packed floats");

namespace {

	constexpr char kMagic[4] = { 'P', 'S', 'E', 'T' };
	constexpr uint32_t kFlagNormals = 1u;

	struct Header
	{
		char     magic[4];
		uint32_t version;
		uint32_t flags;
		uint32_t reserved;
		uint64_t count;
		uint64_t hash;
	};
	static_assert(sizeof(Header) == 32, "binary header layout");

	uint64_t payloadBytes(uint64_t count, bool normals)
	{
		return count * (sizeof(Vector3df) * 2 + sizeof(float)) + (normals ? count * sizeof(Vector3df) : 0);
	}

	template <typename T>
	void writeArray(std::ofstream& ofs, const std::vector<T>& v)
	{
		if (!v.empty()) ofs.write(reinterpret_cast<const char*>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(T)));
	}

	template <typename T>
	bool readArray(std::ifstream& ifs, std::vector<T>& v, uint64_t count)
	{
		v.resize(static_cast<size_t>(count));
		if (count == 0) return true;
		ifs.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(count * sizeof(T)));
		return static_cast<bool>(ifs);
	}

}

std::string Phantom::PC::hashToHex(uint64_t hash)
{
	char buf[17];
	std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(hash));
	return buf;
}

bool Phantom::PC::loadPointSetFromPly(const std::filesystem::path& path, const PointSetImportOptions& options,
	PointSet& out, PointSetImportInfo& info, std::string& error)
{
	error.clear();
	PLYFileReader reader;
	if (!reader.read(path)) {
		error = "cannot read PLY '" + path.string() + "': " + reader.getLastError();
		return false;
	}
	const PLYFile& file = reader.getFile();
	const size_t n = file.size();

	PointSet set;
	set.positions = file.getPoints();

	PointSetImportInfo got;
	got.hadColor = file.getColors().size() == n && n > 0;
	got.hadRadius = file.getRadii().size() == n && n > 0;
	got.hadNormals = file.getNormals().size() == n && n > 0;

	if (got.hadColor) {
		set.colors.reserve(n);
		for (const auto& c : file.getColors())
			set.colors.emplace_back(srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z));
	} else {
		set.colors.assign(n, options.defaultColor);
	}

	if (got.hadRadius) {
		set.radii = file.getRadii();
	} else {
		float radius = options.defaultRadius;
		if (!(radius > 0.0f)) radius = defaultRadiusForBounds(set.bounds(), n);
		set.radii.assign(n, radius);
		got.appliedRadius = radius;
	}

	if (got.hadNormals) set.normals = file.getNormals();

	const std::string problem = set.validate();
	if (!problem.empty()) {
		error = "invalid PLY '" + path.string() + "': " + problem;
		return false;
	}
	out = std::move(set);
	info = got;
	return true;
}

bool Phantom::PC::savePointSetToPly(const std::filesystem::path& path, const PointSet& points, bool binary, std::string& error)
{
	error.clear();
	const std::string problem = points.validate();
	if (!problem.empty()) {
		error = "cannot write PLY: " + problem;
		return false;
	}
	PLYFile file;
	file.getPoints() = points.positions;
	file.getColors().reserve(points.size());
	for (const auto& c : points.colors)
		file.getColors().emplace_back(linearToSrgb(c.x), linearToSrgb(c.y), linearToSrgb(c.z));
	file.getRadii() = points.radii;
	if (points.hasNormals()) file.getNormals() = points.normals;

	PLYFileWriter writer;
	const bool ok = binary ? writer.writeBinary(path, file) : writer.writeAscii(path, file);
	if (!ok) error = "cannot write PLY '" + path.string() + "'";
	return ok;
}

bool Phantom::PC::savePointSetBinary(const std::filesystem::path& path, const PointSet& points, std::string& error)
{
	error.clear();
	const std::string problem = points.validate();
	if (!problem.empty()) {
		error = "cannot save point set: " + problem;
		return false;
	}

	Header h{};
	std::memcpy(h.magic, kMagic, sizeof(kMagic));
	h.version = kPointSetBinaryVersion;
	h.flags = points.hasNormals() ? kFlagNormals : 0u;
	h.count = points.size();
	h.hash = contentHash(points);

	std::filesystem::path tmp = path;
	tmp += ".tmp";
	{
		std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
		if (!ofs) {
			error = "cannot write '" + tmp.string() + "'";
			return false;
		}
		ofs.write(reinterpret_cast<const char*>(&h), sizeof(h));
		writeArray(ofs, points.positions);
		writeArray(ofs, points.colors);
		writeArray(ofs, points.radii);
		writeArray(ofs, points.normals);
		ofs.flush();
		if (!ofs) {
			error = "write failed for '" + tmp.string() + "'";
			ofs.close();
			std::error_code ec;
			std::filesystem::remove(tmp, ec);
			return false;
		}
	}
	std::error_code ec;
	std::filesystem::rename(tmp, path, ec);
	if (ec) {
		std::filesystem::remove(tmp, ec);
		error = "cannot move the point set into place at '" + path.string() + "'";
		return false;
	}
	return true;
}

bool Phantom::PC::loadPointSetBinary(const std::filesystem::path& path, PointSet& out, uint64_t& hash, std::string& error)
{
	error.clear();
	std::error_code ec;
	const uint64_t fileSize = std::filesystem::file_size(path, ec);
	if (ec) {
		error = "point set file not found: " + path.string();
		return false;
	}
	std::ifstream ifs(path, std::ios::binary);
	if (!ifs) {
		error = "cannot open '" + path.string() + "'";
		return false;
	}
	Header h{};
	ifs.read(reinterpret_cast<char*>(&h), sizeof(h));
	if (!ifs || std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0) {
		error = "'" + path.string() + "' is not a point set file";
		return false;
	}
	if (h.version == 0 || h.version > kPointSetBinaryVersion) {
		error = "'" + path.string() + "' has point set format version " + std::to_string(h.version)
			+ " (this build reads up to " + std::to_string(kPointSetBinaryVersion) + ")";
		return false;
	}
	const bool normals = (h.flags & kFlagNormals) != 0;
	// Check the size against the declared count before allocating, so a corrupt count cannot
	// request an enormous buffer.
	constexpr uint64_t kMaxCount = 1ull << 32;
	if (h.count > kMaxCount || fileSize != sizeof(Header) + payloadBytes(h.count, normals)) {
		error = "'" + path.string() + "' is truncated or corrupt (size does not match its point count)";
		return false;
	}

	PointSet set;
	if (!readArray(ifs, set.positions, h.count) || !readArray(ifs, set.colors, h.count)
		|| !readArray(ifs, set.radii, h.count) || (normals && !readArray(ifs, set.normals, h.count))) {
		error = "failed to read '" + path.string() + "'";
		return false;
	}
	if (contentHash(set) != h.hash) {
		error = "'" + path.string() + "' is corrupt (content hash mismatch)";
		return false;
	}
	const std::string problem = set.validate();
	if (!problem.empty()) {
		error = "'" + path.string() + "' holds an invalid point set: " + problem;
		return false;
	}
	out = std::move(set);
	hash = h.hash;
	return true;
}
