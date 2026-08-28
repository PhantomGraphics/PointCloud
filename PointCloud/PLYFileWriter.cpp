#include "PLYFileWriter.h"

#include <fstream>
#include <iomanip>
#include <sstream>

using namespace Phantom::PC;

bool PLYFileWriter::writeAscii(const std::filesystem::path& filename, const PLYFile& file)
{
	const auto& points = file.getPoints();

	std::ofstream ofs(filename);
	if (!ofs) return false;

	ofs << "ply\n";
	ofs << "format ascii 1.0\n";
	ofs << "element vertex " << points.size() << "\n";
	ofs << "property float x\n";
	ofs << "property float y\n";
	ofs << "property float z\n";
	ofs << "end_header\n";

	ofs << std::fixed << std::setprecision(6);
	for (const auto& p : points) {
		ofs << p.x << " " << p.y << " " << p.z << "\n";
	}

	return ofs.good();
}

bool PLYFileWriter::writeBinary(const std::filesystem::path& filename, const PLYFile& file)
{
	const auto& points = file.getPoints();

	std::ofstream ofs(filename, std::ios::binary);
	if (!ofs) return false;

	std::ostringstream header;
	header << "ply\n";
	header << "format binary_little_endian 1.0\n";
	header << "element vertex " << points.size() << "\n";
	header << "property float x\n";
	header << "property float y\n";
	header << "property float z\n";
	header << "end_header\n";

	const std::string hdr = header.str();
	ofs.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));

	for (const auto& p : points) {
		ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
		ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
		ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
		if (!ofs) return false;
	}

	return true;
}
