#include "PointCloudFileLoader.h"
#include "PLYFileReader.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace Phantom { namespace PC {

namespace {

static Math::Vector3df defaultColor() { return { 0.6f, 0.8f, 1.0f }; }

static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static bool loadPCDImpl(const std::string& path, PointCloudColoredData& result, std::string& err) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) { err = "Cannot open: " + path; return false; }

    std::string line;
    bool inData = false, hasColor = false, asciiData = false;

    while (std::getline(ifs, line)) {
        if (!inData) {
            if (line.find("FIELDS") == 0 &&
                (line.find("rgb") != std::string::npos ||
                 line.find("r g b") != std::string::npos)) {
                hasColor = true;
            }
            if (line.find("DATA ascii") == 0) {
                inData = true; asciiData = true; continue;
            }
            if (line.find("DATA binary") == 0) {
                err = "Binary PCD not supported: " + path;
                return false;
            }
            continue;
        }
        if (!asciiData) break;

        std::istringstream ss(line);
        float x = 0.f, y = 0.f, z = 0.f;
        if (!(ss >> x >> y >> z)) continue;

        Math::Vector3df color = defaultColor();
        if (hasColor) {
            float r = 0.f, g = 0.f, b = 0.f;
            ss >> r;
            if (ss >> g >> b) {
                if (r > 1.f || g > 1.f || b > 1.f) { r /= 255.f; g /= 255.f; b /= 255.f; }
                color = { r, g, b };
            } else {
                uint32_t packed = 0;
                std::memcpy(&packed, &r, sizeof(uint32_t));
                color = {
                    ((packed >> 16) & 0xFF) / 255.f,
                    ((packed >>  8) & 0xFF) / 255.f,
                    ( packed        & 0xFF) / 255.f
                };
            }
        }
        result.positions.push_back({ x, y, z });
        result.colors.push_back(color);
    }
    return true;
}

static bool loadTXTImpl(const std::string& path, PointCloudColoredData& result, std::string& err) {
    std::ifstream ifs(path);
    if (!ifs) { err = "Cannot open: " + path; return false; }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        float x = 0.f, y = 0.f, z = 0.f;
        if (!(ss >> x >> y >> z)) continue;

        Math::Vector3df color = defaultColor();
        float r = 0.f, g = 0.f, b = 0.f;
        if (ss >> r >> g >> b) {
            if (r > 1.f || g > 1.f || b > 1.f) { r /= 255.f; g /= 255.f; b /= 255.f; }
            color = { r, g, b };
        }
        result.positions.push_back({ x, y, z });
        result.colors.push_back(color);
    }
    return true;
}

static bool loadPLYImpl(const std::string& path, PointCloudColoredData& result, std::string& err) {
    Phantom::PC::PLYFileReader reader;
    if (!reader.read(std::filesystem::path(path))) {
        err = reader.getLastError();
        return false;
    }
    const auto& points = reader.getFile().getPoints();
    result.positions.reserve(points.size());
    result.colors.reserve(points.size());
    const Math::Vector3df dc = defaultColor();
    for (const auto& p : points) {
        result.positions.push_back(p);
        result.colors.push_back(dc);
    }
    return true;
}

static bool saveTXTImpl(const std::string& path, const PointCloudColoredData& data, std::string& err) {
    std::ofstream ofs(path);
    if (!ofs) { err = "Cannot write: " + path; return false; }
    for (size_t i = 0; i < data.positions.size(); ++i) {
        const auto& p = data.positions[i];
        const auto& c = data.colors[i];
        ofs << p.x << " " << p.y << " " << p.z << " "
            << c.x << " " << c.y << " " << c.z << "\n";
    }
    return true;
}

static bool savePCDImpl(const std::string& path, const PointCloudColoredData& data, std::string& err) {
    std::ofstream ofs(path);
    if (!ofs) { err = "Cannot write: " + path; return false; }
    ofs << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        << "FIELDS x y z r g b\nSIZE 4 4 4 4 4 4\n"
        << "TYPE F F F F F F\nCOUNT 1 1 1 1 1 1\n"
        << "WIDTH " << data.size() << "\nHEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << data.size() << "\nDATA ascii\n";
    for (size_t i = 0; i < data.positions.size(); ++i) {
        const auto& p = data.positions[i]; const auto& c = data.colors[i];
        ofs << p.x << " " << p.y << " " << p.z << " "
            << c.x << " " << c.y << " " << c.z << "\n";
    }
    return true;
}

static bool savePLYImpl(const std::string& path, const PointCloudColoredData& data, std::string& err) {
    std::ofstream ofs(path);
    if (!ofs) { err = "Cannot write: " + path; return false; }
    ofs << "ply\nformat ascii 1.0\nelement vertex " << data.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (size_t i = 0; i < data.positions.size(); ++i) {
        const auto& p = data.positions[i]; const auto& c = data.colors[i];
        const int r = static_cast<int>(std::clamp(c.x, 0.f, 1.f) * 255.f);
        const int g = static_cast<int>(std::clamp(c.y, 0.f, 1.f) * 255.f);
        const int b = static_cast<int>(std::clamp(c.z, 0.f, 1.f) * 255.f);
        ofs << p.x << " " << p.y << " " << p.z << " " << r << " " << g << " " << b << "\n";
    }
    return true;
}

} // anonymous namespace

bool loadPointCloud(const std::string& path, PointCloudColoredData& out, std::string& errorMessage) {
    errorMessage.clear();
    out = {};
    const std::string ext = toLower(std::filesystem::path(path).extension().string());
    if (ext == ".pcd") return loadPCDImpl(path, out, errorMessage);
    if (ext == ".ply") return loadPLYImpl(path, out, errorMessage);
    if (ext == ".txt") return loadTXTImpl(path, out, errorMessage);
    errorMessage = "Unsupported extension: " + ext + " (supported: .pcd, .ply, .txt)";
    return false;
}

bool savePointCloud(const std::string& path, const PointCloudColoredData& data, std::string& errorMessage) {
    errorMessage.clear();
    const std::string ext = toLower(std::filesystem::path(path).extension().string());
    if (ext == ".pcd") return savePCDImpl(path, data, errorMessage);
    if (ext == ".ply") return savePLYImpl(path, data, errorMessage);
    if (ext == ".txt") return saveTXTImpl(path, data, errorMessage);
    errorMessage = "Unsupported extension: " + ext + " (supported: .pcd, .ply, .txt)";
    return false;
}

}} // namespace Phantom::PC
