#include "TXTFileReader.h"

#include <sstream>
#include <fstream>
#include <charconv>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	std::vector<std::string> split(const std::string& input, char delimiter)
	{
		std::istringstream stream(input);

		std::string field;
		std::vector<std::string> result;
		while (std::getline(stream, field, delimiter)) {
			result.push_back(field);
		}
		return result;
	}

	bool parseFloatToken(const std::string& s, float& out)
	{
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}
}

bool TXTFileReader::read(const std::filesystem::path& filename)
{
  positions.clear();
	lastError_.clear();
	std::fstream stream(filename);
	if (!stream.is_open()) {
       lastError_ = "Failed to open file: " + filename.string();
		return false;
	}
	const auto res = read(stream);
	return res;
}

bool TXTFileReader::read(std::istream& stream)
{
    positions.clear();
	lastError_.clear();
	std::string str;
 size_t lineNo = 0;
	while (std::getline(stream, str)) {
        ++lineNo;
		const auto strs = ::split(str, ' ');
		if (strs.size() < 3) {
			continue;
		}
		float x, y, z;
		if (!parseFloatToken(strs[0], x) || !parseFloatToken(strs[1], y) || !parseFloatToken(strs[2], z)) {
			lastError_ = "Failed to parse point at line " + std::to_string(lineNo) + ".";
			return false;
		}
		positions.emplace_back(x, y, z);
	}
	return true;
}
