#include "PLYFileWriter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace Phantom::PC;

namespace {

	// Optional attributes are written only when they cover every point; a partial array would
	// misalign the rows, so it is treated as absent.
	struct Layout {
		bool color = false, normal = false, radius = false;
	};

	Layout layoutOf(const PLYFile& file)
	{
		const size_t n = file.size();
		Layout l;
		l.color  = n > 0 && file.getColors().size()  == n;
		l.normal = n > 0 && file.getNormals().size() == n;
		l.radius = n > 0 && file.getRadii().size()   == n;
		return l;
	}

	std::string header(const char* format, size_t count, const Layout& l)
	{
		std::ostringstream h;
		h << "ply\n";
		h << "format " << format << " 1.0\n";
		h << "element vertex " << count << "\n";
		h << "property float x\n";
		h << "property float y\n";
		h << "property float z\n";
		if (l.normal) h << "property float nx\nproperty float ny\nproperty float nz\n";
		if (l.color)  h << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
		if (l.radius) h << "property float radius\n";
		h << "end_header\n";
		return h.str();
	}

	uint8_t toByte(float v)
	{
		if (!(v > 0.0f)) return 0;   // also maps NaN to 0
		if (v >= 1.0f) return 255;
		return static_cast<uint8_t>(std::lround(v * 255.0f));
	}

}

bool PLYFileWriter::writeAscii(const std::filesystem::path& filename, const PLYFile& file)
{
	const auto& points = file.getPoints();
	const Layout l = layoutOf(file);

	std::ofstream ofs(filename);
	if (!ofs) return false;

	ofs << header("ascii", points.size(), l);

	ofs << std::fixed << std::setprecision(6);
	for (size_t i = 0; i < points.size(); ++i) {
		const auto& p = points[i];
		ofs << p.x << " " << p.y << " " << p.z;
		if (l.normal) {
			const auto& n = file.getNormals()[i];
			ofs << " " << n.x << " " << n.y << " " << n.z;
		}
		if (l.color) {
			const auto& c = file.getColors()[i];
			ofs << " " << static_cast<int>(toByte(c.x)) << " " << static_cast<int>(toByte(c.y))
				<< " " << static_cast<int>(toByte(c.z));
		}
		if (l.radius) ofs << " " << file.getRadii()[i];
		ofs << "\n";
	}

	return ofs.good();
}

bool PLYFileWriter::writeBinary(const std::filesystem::path& filename, const PLYFile& file)
{
	const auto& points = file.getPoints();
	const Layout l = layoutOf(file);

	std::ofstream ofs(filename, std::ios::binary);
	if (!ofs) return false;

	const std::string hdr = header("binary_little_endian", points.size(), l);
	ofs.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));

	const size_t rowBytes = 12 + (l.normal ? 12 : 0) + (l.color ? 3 : 0) + (l.radius ? 4 : 0);
	const size_t rowsPerChunk = std::max<size_t>(1, (1u << 20) / rowBytes);
	std::vector<char> buffer(rowsPerChunk * rowBytes);

	size_t i = 0;
	while (i < points.size()) {
		const size_t rows = std::min(rowsPerChunk, points.size() - i);
		char* out = buffer.data();
		for (size_t r = 0; r < rows; ++r, ++i) {
			const auto& p = points[i];
			std::memcpy(out, &p.x, 4); std::memcpy(out + 4, &p.y, 4); std::memcpy(out + 8, &p.z, 4);
			out += 12;
			if (l.normal) {
				const auto& n = file.getNormals()[i];
				std::memcpy(out, &n.x, 4); std::memcpy(out + 4, &n.y, 4); std::memcpy(out + 8, &n.z, 4);
				out += 12;
			}
			if (l.color) {
				const auto& c = file.getColors()[i];
				out[0] = static_cast<char>(toByte(c.x));
				out[1] = static_cast<char>(toByte(c.y));
				out[2] = static_cast<char>(toByte(c.z));
				out += 3;
			}
			if (l.radius) {
				std::memcpy(out, &file.getRadii()[i], 4);
				out += 4;
			}
		}
		ofs.write(buffer.data(), static_cast<std::streamsize>(rows * rowBytes));
		if (!ofs) return false;
	}

	return true;
}
