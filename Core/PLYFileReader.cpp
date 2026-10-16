#include "PLYFileReader.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <charconv>

using namespace Phantom::PC;
using namespace Phantom::Math;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

	enum class PLYFormat { ASCII, BINARY_LITTLE, BINARY_BIG, UNKNOWN };
	using PLYType = Phantom::PC::detail::PLYType;

	struct PLYProp {
		std::string name;
		PLYType     type      = PLYType::UNKNOWN;
		bool        isList    = false;   // property list 窶・skip when reading vertex data
	};

	struct PLYElement {
		std::string           name;
		size_t                count = 0;
		std::vector<PLYProp>  props;
		bool                  hasListProp = false;
	};

	static inline std::string toLower(std::string s) {
		std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
		return s;
	}

	static inline std::string trimLine(const std::string& s) {
		const auto b = s.find_first_not_of(" \t\r\n");
		if (b == std::string::npos) return {};
		const auto e = s.find_last_not_of(" \t\r\n");
		return s.substr(b, e - b + 1);
	}

	static inline std::vector<std::string> splitTokens(const std::string& s) {
		std::istringstream iss(s);
		std::vector<std::string> out;
		std::string tok;
		while (iss >> tok) out.push_back(tok);
		return out;
	}

	static PLYType parsePLYType(const std::string& t) {
		const auto s = toLower(t);
		if (s == "float"   || s == "float32") return PLYType::FLOAT32;
		if (s == "double"  || s == "float64") return PLYType::FLOAT64;
		if (s == "char"    || s == "int8"   ) return PLYType::INT8;
		if (s == "uchar"   || s == "uint8"  ) return PLYType::UINT8;
		if (s == "short"   || s == "int16"  ) return PLYType::INT16;
		if (s == "ushort"  || s == "uint16" ) return PLYType::UINT16;
		if (s == "int"     || s == "int32"  ) return PLYType::INT32;
		if (s == "uint"    || s == "uint32" ) return PLYType::UINT32;
		return PLYType::UNKNOWN;
	}

	static size_t typeSize(PLYType t) {
		switch (t) {
		case PLYType::FLOAT32: return 4;
		case PLYType::FLOAT64: return 8;
		case PLYType::INT8:
		case PLYType::UINT8:   return 1;
		case PLYType::INT16:
		case PLYType::UINT16:  return 2;
		case PLYType::INT32:
		case PLYType::UINT32:  return 4;
		default:               return 0;
		}
	}

	// Swap 4 bytes for big-endian 竊・little-endian conversion
	static inline float swapFloat32(float v) {
		uint32_t u;
		std::memcpy(&u, &v, 4);
		u = ((u & 0xFF000000u) >> 24) | ((u & 0x00FF0000u) >> 8)
		  | ((u & 0x0000FF00u) << 8)  | ((u & 0x000000FFu) << 24);
		std::memcpy(&v, &u, 4);
		return v;
	}

	static inline double swapFloat64(double v) {
		uint64_t u;
		std::memcpy(&u, &v, 8);
		u = ((u & 0xFF00000000000000ull) >> 56) | ((u & 0x00FF000000000000ull) >> 40)
		  | ((u & 0x0000FF0000000000ull) >> 24) | ((u & 0x000000FF00000000ull) >> 8)
		  | ((u & 0x00000000FF000000ull) << 8)  | ((u & 0x0000000000FF0000ull) << 24)
		  | ((u & 0x000000000000FF00ull) << 40) | ((u & 0x00000000000000FFull) << 56);
		std::memcpy(&v, &u, 8);
		return v;
	}

	static inline uint16_t swapUint16(uint16_t v) {
		return static_cast<uint16_t>((v >> 8) | (v << 8));
	}

	static inline uint32_t swapUint32(uint32_t v) {
		return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >> 8)
			| ((v & 0x0000FF00u) << 8) | ((v & 0x000000FFu) << 24);
	}

	static inline bool parseSizeToken(const std::string& s, size_t& out) {
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}

	static inline bool parseFloatToken(const std::string& s, float& out) {
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}

	// Parse PLY header from stream.
	// On success returns true and fills elements and format.
	// Stream is positioned just after the "end_header" line.
	static bool parseHeader(std::istream& stream,
		std::vector<PLYElement>& elements, PLYFormat& format)
	{
		std::string line;
		if (!std::getline(stream, line) || trimLine(line) != "ply") return false;

		format = PLYFormat::UNKNOWN;
		PLYElement* current = nullptr;

		while (std::getline(stream, line)) {
			const auto t = trimLine(line);
			if (t.empty() || t[0] == '#') continue;
			if (t == "end_header") break;

			const auto tokens = splitTokens(t);
			if (tokens.empty()) continue;
			const auto& key = tokens[0];

			if (key == "format" && tokens.size() >= 2) {
				const auto f = toLower(tokens[1]);
				if      (f == "ascii")                  format = PLYFormat::ASCII;
				else if (f == "binary_little_endian")   format = PLYFormat::BINARY_LITTLE;
				else if (f == "binary_big_endian")      format = PLYFormat::BINARY_BIG;
			}
			else if (key == "element" && tokens.size() >= 3) {
				size_t count;
				if (!parseSizeToken(tokens[2], count)) {
					return false;
				}
				elements.push_back({ tokens[1], count, {} });
				current = &elements.back();
			}
			else if (key == "property" && current != nullptr) {
				if (tokens.size() >= 2 && toLower(tokens[1]) == "list") {
					// property list <count_type> <value_type> <name>
					PLYProp p;
					p.isList = true;
					p.name   = (tokens.size() >= 5) ? tokens[4] : "";
					current->props.push_back(p);
					current->hasListProp = true;
				}
				else if (tokens.size() >= 3) {
					// property <type> <name>
					PLYProp p;
					p.type = parsePLYType(tokens[1]);
					p.name = tokens[2];
					current->props.push_back(p);
				}
			}
		}

		return format != PLYFormat::UNKNOWN && !elements.empty();
	}

	// Find the vertex element and extract the indices of the properties we understand.
	// Returns false if the vertex element or its x/y/z properties are not found.
	static bool findVertexInfo(const std::vector<PLYElement>& elements,
		size_t& vertexCount,
		Phantom::PC::detail::PLYVertexFields& fields,
		const PLYElement** vertexElem)
	{
		const PLYElement* ve = nullptr;
		for (const auto& e : elements) {
			if (e.name == "vertex") { ve = &e; break; }
		}
		if (!ve) return false;

		*vertexElem = ve;
		vertexCount = ve->count;

		for (size_t i = 0; i < ve->props.size(); ++i) {
			if (ve->props[i].isList) continue;
			const auto n = toLower(ve->props[i].name);
			if      (n == "x")  fields.x  = i;
			else if (n == "y")  fields.y  = i;
			else if (n == "z")  fields.z  = i;
			else if (n == "red"   || n == "r") fields.r = i;
			else if (n == "green" || n == "g") fields.g = i;
			else if (n == "blue"  || n == "b") fields.b = i;
			else if (n == "nx") fields.nx = i;
			else if (n == "ny") fields.ny = i;
			else if (n == "nz") fields.nz = i;
		}
		return fields.hasPosition();
	}

	// Scale that maps a color channel of the given type onto [0,1].
	// Integer channels carry 0..typeMax (uchar 0-255 is by far the most common in the
	// wild); float channels are assumed to be normalized already, which is what the
	// float-color PLY writers seen in practice emit.
	static float colorScale(PLYType t) {
		switch (t) {
		case PLYType::UINT8:
		case PLYType::INT8:   return 1.0f / 255.0f;
		case PLYType::UINT16:
		case PLYType::INT16:  return 1.0f / 65535.0f;
		default:              return 1.0f;
		}
	}

} // anonymous namespace

// ---------------------------------------------------------------------------
// PLYFileReader
// ---------------------------------------------------------------------------

bool PLYFileReader::read(const std::filesystem::path& filename)
{
	file.clear();
	lastError_.clear();

	std::ifstream ifs(filename, std::ios::binary);
   if (!ifs.is_open()) {
		lastError_ = "Failed to open file: " + filename.string();
		return false;
	}

	std::vector<PLYElement> elements;
	PLYFormat fmt = PLYFormat::UNKNOWN;
	if (!parseHeader(ifs, elements, fmt)) {
		lastError_ = "Failed to parse PLY header.";
		return false;
	}

	size_t vertexCount = 0;
	Phantom::PC::detail::PLYVertexFields fields;
	const PLYElement* vertexElem = nullptr;
	if (!findVertexInfo(elements, vertexCount, fields, &vertexElem)) {
		lastError_ = "Vertex element with x/y/z properties was not found.";
		return false;
	}

	// Build per-property size and type vectors (scalar props only)
	std::vector<size_t> propSizes;
	std::vector<PLYType> propTypes;
	for (const auto& p : vertexElem->props) {
		if (p.isList) {
			// list property inside vertex - not supported for field extraction
			lastError_ = "List property in vertex element is not supported.";
			return false;
		}
		if (typeSize(p.type) == 0) {
			lastError_ = "Unsupported vertex property type in PLY header.";
			return false;
		}
		propSizes.push_back(typeSize(p.type));
		propTypes.push_back(p.type);
	}

	if (fmt == PLYFormat::ASCII) {
		return readASCII(ifs, fields, propTypes, vertexElem->props.size(), vertexCount);
	}
	return readBinary(ifs, propSizes, propTypes, fields, vertexCount,
		fmt == PLYFormat::BINARY_BIG);
}

bool PLYFileReader::readASCII(std::istream& stream,
	const detail::PLYVertexFields& fields,
	const std::vector<PLYType>& propTypes,
	size_t propCount, size_t vertexCount)
{
	auto& points  = file.getPoints();
	auto& colors  = file.getColors();
	auto& normals = file.getNormals();
	points.reserve(vertexCount);

	const bool wantColor  = fields.hasColor();
	const bool wantNormal = fields.hasNormal();
	if (wantColor)  colors.reserve(vertexCount);
	if (wantNormal) normals.reserve(vertexCount);

	const float rScale = wantColor ? colorScale(propTypes[fields.r]) : 1.0f;
	const float gScale = wantColor ? colorScale(propTypes[fields.g]) : 1.0f;
	const float bScale = wantColor ? colorScale(propTypes[fields.b]) : 1.0f;

	std::string line;
	size_t read = 0;
	size_t lineNo = 0;
	while (read < vertexCount && std::getline(stream, line)) {
		++lineNo;
		const auto t = trimLine(line);
		if (t.empty() || t[0] == '#') continue;

		const auto tokens = splitTokens(t);
		if (tokens.size() < propCount) {
			lastError_ = "Invalid ASCII vertex line at line " + std::to_string(lineNo) + ".";
			return false;
		}

		float x, y, z;
		if (!parseFloatToken(tokens[fields.x], x)
			|| !parseFloatToken(tokens[fields.y], y)
			|| !parseFloatToken(tokens[fields.z], z)) {
			lastError_ = "Failed to parse ASCII vertex at line " + std::to_string(lineNo) + ".";
			return false;
		}
		points.emplace_back(x, y, z);

		if (wantColor) {
			float r = 0.0f, g = 0.0f, b = 0.0f;
			if (!parseFloatToken(tokens[fields.r], r)
				|| !parseFloatToken(tokens[fields.g], g)
				|| !parseFloatToken(tokens[fields.b], b)) {
				lastError_ = "Failed to parse ASCII vertex color at line " + std::to_string(lineNo) + ".";
				return false;
			}
			colors.emplace_back(r * rScale, g * gScale, b * bScale);
		}
		if (wantNormal) {
			float nx = 0.0f, ny = 0.0f, nz = 0.0f;
			if (!parseFloatToken(tokens[fields.nx], nx)
				|| !parseFloatToken(tokens[fields.ny], ny)
				|| !parseFloatToken(tokens[fields.nz], nz)) {
				lastError_ = "Failed to parse ASCII vertex normal at line " + std::to_string(lineNo) + ".";
				return false;
			}
			normals.emplace_back(nx, ny, nz);
		}
		++read;
	}
	if (read != vertexCount) {
		lastError_ = "Unexpected end of ASCII vertex data.";
		return false;
	}
	return true;
}

// Helper: read one scalar property from buffer at given byte offset
namespace {
  static inline float readPropAsFloat(const char* buf, size_t off, PLYType type, bool swapBytes) {
		switch (type) {
		case PLYType::FLOAT32: {
			float v;
			std::memcpy(&v, buf + off, 4);
			if (swapBytes) v = swapFloat32(v);
			return v;
		}
		case PLYType::FLOAT64: {
			double v;
			std::memcpy(&v, buf + off, 8);
			if (swapBytes) v = swapFloat64(v);
			return static_cast<float>(v);
		}
		case PLYType::INT8: {
			int8_t v;
			std::memcpy(&v, buf + off, 1);
			return static_cast<float>(v);
		}
		case PLYType::UINT8: {
			uint8_t v;
			std::memcpy(&v, buf + off, 1);
			return static_cast<float>(v);
		}
		case PLYType::INT16: {
			int16_t v;
			std::memcpy(&v, buf + off, 2);
			if (swapBytes) {
				uint16_t u;
				std::memcpy(&u, &v, 2);
				u = swapUint16(u);
				std::memcpy(&v, &u, 2);
			}
			return static_cast<float>(v);
		}
		case PLYType::UINT16: {
			uint16_t v;
			std::memcpy(&v, buf + off, 2);
			if (swapBytes) v = swapUint16(v);
			return static_cast<float>(v);
		}
		case PLYType::INT32: {
			int32_t v;
			std::memcpy(&v, buf + off, 4);
			if (swapBytes) {
				uint32_t u;
				std::memcpy(&u, &v, 4);
				u = swapUint32(u);
				std::memcpy(&v, &u, 4);
			}
			return static_cast<float>(v);
		}
		case PLYType::UINT32: {
			uint32_t v;
			std::memcpy(&v, buf + off, 4);
			if (swapBytes) v = swapUint32(v);
			return static_cast<float>(v);
		}
		default:
			return 0.0f;
		}
	}
}

bool PLYFileReader::readBinary(std::istream& stream,
	const std::vector<size_t>& propSizes,
	const std::vector<PLYType>& propTypes,
	const detail::PLYVertexFields& fields,
	size_t vertexCount, bool swapBytes)
{
	// Compute per-property byte offsets and total row size
	const size_t n = propSizes.size();
	if (n == 0) {
		lastError_ = "Vertex element has no properties.";
		return false;
	}
	std::vector<size_t> offsets(n, 0);
	for (size_t i = 1; i < n; ++i) offsets[i] = offsets[i - 1] + propSizes[i - 1];
	const size_t rowBytes = offsets[n - 1] + propSizes[n - 1];

	auto& points  = file.getPoints();
	auto& colors  = file.getColors();
	auto& normals = file.getNormals();
	points.reserve(vertexCount);

	const bool wantColor  = fields.hasColor();
	const bool wantNormal = fields.hasNormal();
	if (wantColor)  colors.reserve(vertexCount);
	if (wantNormal) normals.reserve(vertexCount);

	const float rScale = wantColor ? colorScale(propTypes[fields.r]) : 1.0f;
	const float gScale = wantColor ? colorScale(propTypes[fields.g]) : 1.0f;
	const float bScale = wantColor ? colorScale(propTypes[fields.b]) : 1.0f;

	// Read many rows per istream::read() call. Scan data sets reach tens of millions of
	// vertices, where one read() per vertex is dominated by stream call overhead rather
	// than by the actual copy.
	const size_t rowsPerChunk = std::max<size_t>(1, (1u << 20) / rowBytes);
	std::vector<char> buffer(rowsPerChunk * rowBytes);

	size_t remaining = vertexCount;
	while (remaining > 0) {
		const size_t rows = std::min(rowsPerChunk, remaining);
		stream.read(buffer.data(), static_cast<std::streamsize>(rows * rowBytes));
		if (!stream) {
			lastError_ = "Unexpected end of binary vertex data.";
			return false;
		}
		for (size_t i = 0; i < rows; ++i) {
			const char* row = buffer.data() + i * rowBytes;
			points.emplace_back(
				readPropAsFloat(row, offsets[fields.x], propTypes[fields.x], swapBytes),
				readPropAsFloat(row, offsets[fields.y], propTypes[fields.y], swapBytes),
				readPropAsFloat(row, offsets[fields.z], propTypes[fields.z], swapBytes));
			if (wantColor) {
				colors.emplace_back(
					readPropAsFloat(row, offsets[fields.r], propTypes[fields.r], swapBytes) * rScale,
					readPropAsFloat(row, offsets[fields.g], propTypes[fields.g], swapBytes) * gScale,
					readPropAsFloat(row, offsets[fields.b], propTypes[fields.b], swapBytes) * bScale);
			}
			if (wantNormal) {
				normals.emplace_back(
					readPropAsFloat(row, offsets[fields.nx], propTypes[fields.nx], swapBytes),
					readPropAsFloat(row, offsets[fields.ny], propTypes[fields.ny], swapBytes),
					readPropAsFloat(row, offsets[fields.nz], propTypes[fields.nz], swapBytes));
			}
		}
		remaining -= rows;
	}
	return true;
}
