#include "PCDFileReader.h"
#include "PCDFileReader.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <charconv>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {
	static inline std::string trim(const std::string& s) {
		auto b = s.find_first_not_of(" \t\r\n");
		if (b == std::string::npos) return {};
		auto e = s.find_last_not_of(" \t\r\n");
		return s.substr(b, e - b + 1);
	}

	static inline std::vector<std::string> split(const std::string& s) {
		std::istringstream iss(s);
		std::vector<std::string> out;
		std::string tok;
		while (iss >> tok) out.push_back(tok);
		return out;
	}

	// フィールド名から x,y,z のインデックスを探す（存在しなければ -1）
	static inline int findFieldIndex(const std::vector<std::string>& fields, const std::string& name) {
		for (size_t i = 0; i < fields.size(); ++i) {
			if (fields[i] == name) return static_cast<int>(i);
		}
		return -1;
	}

	static inline bool parseIntToken(const std::string& s, int& out) {
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}

	static inline bool parseSizeToken(const std::string& s, size_t& out) {
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}

	static inline bool parseFloatToken(const std::string& s, float& out) {
		const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
		return ec == std::errc{};
	}
}

bool PCDFileReader::read(const std::string& filename)
{
	this->file.clear();
	this->lastError_.clear();

	std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
		this->lastError_ = "Failed to open file: " + filename;
		return false;
	}

	std::string line;
	std::vector<std::string> fields;
	std::vector<int> sizes;
	std::vector<char> types;
	std::vector<int> counts;
	size_t points = 0;
	std::string dataType;

	while (std::getline(ifs, line)) {
		auto t = trim(line);
		if (t.empty()) continue;
		if (t[0] == '#') continue;

		auto tokens = split(t);
		if (tokens.empty()) continue;

		const auto key = tokens[0];
		if (key == "FIELDS" || key == "FIELD") {
			fields.clear();
			for (size_t i = 1; i < tokens.size(); ++i) fields.push_back(tokens[i]);
		}
		else if (key == "SIZE") {
			sizes.clear();
			for (size_t i = 1; i < tokens.size(); ++i) {
				int v;
				if (!parseIntToken(tokens[i], v)) {
					this->lastError_ = "Invalid SIZE value: " + tokens[i];
					return false;
				}
				sizes.push_back(v);
			}
		}
		else if (key == "TYPE") {
			types.clear();
			for (size_t i = 1; i < tokens.size(); ++i) types.push_back(tokens[i][0]);
		}
		else if (key == "COUNT") {
			counts.clear();
			for (size_t i = 1; i < tokens.size(); ++i) {
				int v;
				if (!parseIntToken(tokens[i], v)) {
					this->lastError_ = "Invalid COUNT value: " + tokens[i];
					return false;
				}
				counts.push_back(v);
			}
		}
		else if (key == "POINTS") {
			if (tokens.size() >= 2) {
				size_t v;
				if (!parseSizeToken(tokens[1], v)) {
					this->lastError_ = "Invalid POINTS value: " + tokens[1];
					return false;
				}
				points = v;
			}
		}
		else if (key == "DATA") {
			if (tokens.size() >= 2) dataType = tokens[1];
			break;
		}
	}

	const int idxX = findFieldIndex(fields, "x");
	const int idxY = findFieldIndex(fields, "y");
	const int idxZ = findFieldIndex(fields, "z");
	if (idxX < 0 || idxY < 0 || idxZ < 0) {
		this->lastError_ = "Required FIELDS x/y/z are missing.";
		return false;
	}

	std::transform(dataType.begin(), dataType.end(), dataType.begin(), ::tolower);
	if (dataType == "ascii") {
		return readASCII(ifs, fields, points);
	}
	if (dataType == "binary") {
		return readBinary(ifs, fields, sizes, types, counts, points);
	}

	this->lastError_ = "Unsupported DATA type: " + dataType;
	return false;
}

bool PCDFileReader::readASCII(std::istream& stream, const std::vector<std::string>& fields, size_t pointCount)
{
	this->file.clear();

	std::string line;
	size_t readCount = 0;
	const int idxX = findFieldIndex(fields, "x");
	const int idxY = findFieldIndex(fields, "y");
	const int idxZ = findFieldIndex(fields, "z");

    size_t lineNo = 0;
	while (std::getline(stream, line)) {
		++lineNo;
		auto t = trim(line);
		if (t.empty()) continue;
		// コメント skip
		if (t[0] == '#') continue;

		auto tokens = split(t);
		// 行に含まれるトークン数がフィールド数未満でも最初の3つを解釈できる可能性があるため柔軟に扱う
		// ただし一般的にはトークン数 >= fields.size()
		// トークンが数値であることを期待してパースする
		float vx = 0.0f, vy = 0.0f, vz = 0.0f;
		bool ok = true;
		// ASCII は通常フィールド順に値が並ぶので idx を直接参照する
		if (idxX >= 0 && idxX < static_cast<int>(tokens.size())) ok = parseFloatToken(tokens[idxX], vx) && ok;
		if (idxY >= 0 && idxY < static_cast<int>(tokens.size())) ok = parseFloatToken(tokens[idxY], vy) && ok;
		if (idxZ >= 0 && idxZ < static_cast<int>(tokens.size())) ok = parseFloatToken(tokens[idxZ], vz) && ok;

		if (!ok) {
			// パースエラー行はスキップ
			continue;
		}

		this->file.getPoints().push_back(Vector3df(vx, vy, vz));
		++readCount;

		if (pointCount > 0 && readCount >= pointCount) break;
	}

	// pointCount が指定されている場合は一致をチェックするが厳密性は緩める
	if (pointCount > 0 && this->file.size() != pointCount) {
		// データ不足はエラー扱い
		// ただしファイルが不正なだけの可能性があるため false を返す
		return false;
	}

	return true;
}

bool PCDFileReader::readBinary(std::istream& stream, const std::vector<std::string>& fields, const std::vector<int>& sizes, const std::vector<char>& types, const std::vector<int>& counts, size_t pointCount)
{
	this->file.clear();

	// 簡易実装: fields が少なくとも x,y,z を持ち、各要素が float (SIZE 4 TYPE 'F' COUNT 1) であるケースを扱う
	// それ以外は未実装として失敗させる
	const int idxX = findFieldIndex(fields, "x");
	const int idxY = findFieldIndex(fields, "y");
	const int idxZ = findFieldIndex(fields, "z");
 if (idxX < 0 || idxY < 0 || idxZ < 0) {
		this->lastError_ = "Required FIELDS x/y/z are missing.";
		return false;
	}

	// sizes/types/counts の情報がヘッダで取れていない場合は、フィールド数が3で float 連続と仮定
	if (sizes.empty() || types.empty() || counts.empty()) {
		// 期待: 3 フィールド x,y,z
        if (fields.size() < 3) {
			this->lastError_ = "Insufficient fields for implicit xyz binary layout.";
			return false;
		}
		// バイナリで float x,y,z が並んでいると仮定
		size_t elementBytes = sizeof(float) * 3;
		if (pointCount == 0) {
			// POINTS が与えられていない場合はストリームサイズから推定する
			auto cur = stream.tellg();
			stream.seekg(0, std::ios::end);
			auto end = stream.tellg();
           if (end <= cur) {
				this->lastError_ = "Invalid binary stream range.";
				return false;
			}
			const auto remaining = static_cast<std::streamoff>(end - cur);
			stream.seekg(cur, std::ios::beg);
			pointCount = static_cast<size_t>(remaining) / elementBytes;
		}

		this->file.getPoints().reserve(pointCount);
		for (size_t i = 0; i < pointCount; ++i) {
			float x = 0.0f, y = 0.0f, z = 0.0f;
			stream.read(reinterpret_cast<char*>(&x), sizeof(float));
			stream.read(reinterpret_cast<char*>(&y), sizeof(float));
			stream.read(reinterpret_cast<char*>(&z), sizeof(float));
          if (!stream) {
				this->lastError_ = "Unexpected end of binary point data.";
				return false;
			}
			this->file.getPoints().push_back(Vector3df(x, y, z));
		}
		return true;
	}

	// sizes/types/counts 情報がある場合は各フィールドサイズを積算して1点当たりバイト数を計算して読み取る
	size_t fieldCount = fields.size();
	if (sizes.size() < fieldCount || types.size() < fieldCount || counts.size() < fieldCount) {
		// 不一致
		return false;
	}

	// 各フィールドのバイト数を算出
	std::vector<size_t> bytePerField(fieldCount, 0);
	for (size_t i = 0; i < fieldCount; ++i) {
		bytePerField[i] = static_cast<size_t>(sizes[i]) * static_cast<size_t>(counts[i]);
	}
	size_t bytesPerPoint = 0;
	for (size_t i = 0; i < fieldCount; ++i) bytesPerPoint += bytePerField[i];

	// POINTS が不明ならストリーム残量から推定
	if (pointCount == 0) {
		auto cur = stream.tellg();
		stream.seekg(0, std::ios::end);
		auto end = stream.tellg();
       if (end <= cur) {
			this->lastError_ = "Invalid binary stream range.";
			return false;
		}
		const auto remaining = static_cast<std::streamoff>(end - cur);
		stream.seekg(cur, std::ios::beg);
		pointCount = static_cast<size_t>(remaining) / static_cast<size_t>(bytesPerPoint);
	}

	this->file.getPoints().reserve(pointCount);

	// 各点についてフィールドを読み進める。x,y,z のフィールドオフセットを求める
	std::vector<size_t> offsets(fieldCount, 0);
	for (size_t i = 1; i < fieldCount; ++i) offsets[i] = offsets[i - 1] + bytePerField[i - 1];

	// x,y,z は float 扱いで読み出す。フィールドタイプが 'F' かをチェック（簡易）
	for (size_t pt = 0; pt < pointCount; ++pt) {
		std::vector<char> buffer(bytesPerPoint);
		stream.read(buffer.data(), static_cast<std::streamsize>(bytesPerPoint));
      if (!stream) {
			this->lastError_ = "Unexpected end of binary point data.";
			return false;
		}

		auto readFloatFromField = [&](int fidx)->float {
			const size_t off = offsets[fidx];
			// サイズが4, type 'F' であることを期待
			if (sizes[fidx] == 4 && (types[fidx] == 'F' || types[fidx] == 'f')) {
				float v;
				std::memcpy(&v, buffer.data() + off, sizeof(float));
				return v;
			}
			// もしサイズが8で double の場合を扱う
			if (sizes[fidx] == 8 && (types[fidx] == 'F' || types[fidx] == 'f')) {
				double dv;
				std::memcpy(&dv, buffer.data() + off, sizeof(double));
				return static_cast<float>(dv);
			}
			// 他は変換未実装 -> 0
			return 0.0f;
		};

		const float x = readFloatFromField(idxX);
		const float y = readFloatFromField(idxY);
		const float z = readFloatFromField(idxZ);
		this->file.getPoints().push_back(Vector3df(x, y, z));
	}

	return true;
}