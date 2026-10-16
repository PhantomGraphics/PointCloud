#include "TXTFileWriter.h"

#include <fstream>
#include <iomanip>

bool Phantom::PC::TXTFileWriter::write(const std::filesystem::path& filename)
{
	std::ofstream stream(filename);
	if (!stream.is_open()) {
		return false;
	}

	return write(stream);
}

bool Phantom::PC::TXTFileWriter::write(std::ostream& stream)
{
	if (!stream.good()) {
		return false;
	}

	stream << std::fixed << std::setprecision(6);
	for (const auto& p : positions) {
		stream << p.x << " " << p.y << " " << p.z << "\n";
	}

	return stream.good();
}
