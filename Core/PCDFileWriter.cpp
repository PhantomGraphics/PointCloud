#include "PCDFileWriter.h"

#include <fstream>
#include <iomanip>
#include <sstream>

namespace Phantom {
	namespace PC {

		bool PCDFileWriter::writeAscii(const std::string& filename, const PCDFile& file)
		{
			const auto& points = file.getPoints();

			std::ofstream ofs(filename);
			if (!ofs) return false;

			ofs << "VERSION .7\n";
			ofs << "FIELDS x y z\n";
			ofs << "SIZE 4 4 4\n";
			ofs << "TYPE F F F\n";
			ofs << "COUNT 1 1 1\n";
			ofs << "WIDTH " << points.size() << "\n";
			ofs << "HEIGHT 1\n";
			ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
			ofs << "POINTS " << points.size() << "\n";
			ofs << "DATA ascii\n";

			ofs << std::fixed << std::setprecision(6);
			for (const auto& p : points) {
				ofs << p.x << " " << p.y << " " << p.z << "\n";
			}

			return true;
		}

		bool PCDFileWriter::writeBinary(const std::string& filename, const PCDFile& file)
		{
			const auto& points = file.getPoints();

			std::ofstream ofs(filename, std::ios::binary);
			if (!ofs) return false;

			std::ostringstream header;
			header << "VERSION .7\n";
			header << "FIELDS x y z\n";
			header << "SIZE 4 4 4\n";
			header << "TYPE F F F\n";
			header << "COUNT 1 1 1\n";
			header << "WIDTH " << points.size() << "\n";
			header << "HEIGHT 1\n";
			header << "VIEWPOINT 0 0 0 1 0 0 0\n";
			header << "POINTS " << points.size() << "\n";
			header << "DATA binary\n";

			// write header (text) first
			std::string hdr = header.str();
			ofs.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));

			// then write binary floats (x,y,z) per point
			for (const auto& p : points) {
				ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
				ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
				ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
				if (!ofs) return false;
			}

			return true;
		}

	}
}

