#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "GSPointCloud.h"

using namespace Phantom::PointCloud;

namespace {

struct PropertyDef {
    std::string type;
    std::string name;
};

struct ElementDef {
    std::string name;
    size_t count = 0;
    std::vector<PropertyDef> properties;
};

static bool parseHeader(std::istream& file, std::vector<ElementDef>& elements)
{
    std::string line;
    if (!std::getline(file, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "ply") return false;

    ElementDef* cur = nullptr;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "end_header") return true;

        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "element") {
            elements.emplace_back();
            cur = &elements.back();
            iss >> cur->name >> cur->count;
        } else if (token == "property" && cur) {
            PropertyDef pd;
            iss >> pd.type >> pd.name;
            cur->properties.push_back(std::move(pd));
        }
    }
    return false;
}

static float lerpf(float a, float b, float t) { return a + (b - a) * t; }

static constexpr float kSqrt2 = 1.41421356237f;

static bool readCompressed(std::istream& file,
                           const std::vector<ElementDef>& elements,
                           std::vector<GSPoint>& out)
{
    const ElementDef* chunkElem  = nullptr;
    const ElementDef* vertexElem = nullptr;
    for (const auto& e : elements) {
        if      (e.name == "chunk")  chunkElem  = &e;
        else if (e.name == "vertex") vertexElem = &e;
    }
    if (!chunkElem || !vertexElem) return false;

    const size_t chunkStride  = chunkElem->properties.size();
    const size_t vertexCount  = vertexElem->count;

    std::unordered_map<std::string, size_t> chunkProp;
    for (size_t i = 0; i < chunkStride; ++i)
        chunkProp[chunkElem->properties[i].name] = i;

    std::vector<float>    chunkBuf;
    std::vector<uint32_t> vertexBuf;

    // Read elements in file (header) order
    for (const auto& e : elements) {
        if (e.name == "chunk") {
            const size_t n = e.count * chunkStride;
            chunkBuf.resize(n);
            file.read(reinterpret_cast<char*>(chunkBuf.data()),
                      static_cast<std::streamsize>(n * sizeof(float)));
            if (!file) return false;
        } else if (e.name == "vertex") {
            const size_t n = e.count * 4; // packed_position/rotation/scale/color
            vertexBuf.resize(n);
            file.read(reinterpret_cast<char*>(vertexBuf.data()),
                      static_cast<std::streamsize>(n * sizeof(uint32_t)));
            if (!file) return false;
        } else if (e.name == "sh") {
            // SH coefficients are not used; skip (all properties are uchar)
            const auto shBytes = static_cast<std::streamoff>(
                e.count * e.properties.size());
            file.seekg(shBytes, std::ios::cur);
        }
    }

    auto chunkGet = [&](size_t ci, const char* name) -> float {
        const auto it = chunkProp.find(name);
        if (it == chunkProp.end()) return 0.0f;
        return chunkBuf[ci * chunkStride + it->second];
    };

    out.reserve(vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        const size_t   ci = i / 256;
        const uint32_t pp = vertexBuf[i * 4 + 0];
        const uint32_t pr = vertexBuf[i * 4 + 1];
        const uint32_t ps = vertexBuf[i * 4 + 2];
        const uint32_t pc = vertexBuf[i * 4 + 3];

        GSPoint g{};

        // position (11+10+11 bit)
        const float nz = static_cast<float>( pp        & 0x7FFu) / 2047.0f;
        const float ny = static_cast<float>((pp >> 11) & 0x3FFu) / 1023.0f;
        const float nx = static_cast<float>( pp >> 21           ) / 2047.0f;
        g.x = lerpf(chunkGet(ci, "min_x"), chunkGet(ci, "max_x"), nx);
        g.y = lerpf(chunkGet(ci, "min_y"), chunkGet(ci, "max_y"), ny);
        g.z = lerpf(chunkGet(ci, "min_z"), chunkGet(ci, "max_z"), nz);

        // rotation (2+10+10+10 bit, smallest-3 quaternion)
        const int idx = static_cast<int>(pr >> 30);
        float a = static_cast<float>((pr >> 20) & 0x3FFu) / 1023.0f;
        float b = static_cast<float>((pr >> 10) & 0x3FFu) / 1023.0f;
        float c = static_cast<float>( pr        & 0x3FFu) / 1023.0f;
        a = (a - 0.5f) * kSqrt2;
        b = (b - 0.5f) * kSqrt2;
        c = (c - 0.5f) * kSqrt2;
        const float m2 = 1.0f - a * a - b * b - c * c;
        const float m  = m2 > 0.0f ? std::sqrt(m2) : 0.0f;
        // (x,y,z,w) assignment → rot[0]=w, rot[1]=x, rot[2]=y, rot[3]=z
        switch (idx) {
        case 0: g.rot[0] = m; g.rot[1] = a; g.rot[2] = b; g.rot[3] = c; break; // (x,y,z,w)=(a,b,c,m)
        case 1: g.rot[0] = a; g.rot[1] = m; g.rot[2] = b; g.rot[3] = c; break; // (x,y,z,w)=(m,b,c,a)
        case 2: g.rot[0] = a; g.rot[1] = b; g.rot[2] = m; g.rot[3] = c; break; // (x,y,z,w)=(b,m,c,a)
        default:g.rot[0] = a; g.rot[1] = b; g.rot[2] = c; g.rot[3] = m; break; // (x,y,z,w)=(b,c,m,a)
        }

        // scale (11+10+11 bit, same layout as position)
        const float sz = static_cast<float>( ps        & 0x7FFu) / 2047.0f;
        const float sy = static_cast<float>((ps >> 11) & 0x3FFu) / 1023.0f;
        const float sx = static_cast<float>( ps >> 21           ) / 2047.0f;
        g.scale[0] = lerpf(chunkGet(ci, "min_scale_x"), chunkGet(ci, "max_scale_x"), sx);
        g.scale[1] = lerpf(chunkGet(ci, "min_scale_y"), chunkGet(ci, "max_scale_y"), sy);
        g.scale[2] = lerpf(chunkGet(ci, "min_scale_z"), chunkGet(ci, "max_scale_z"), sz);

        // color + opacity (8+8+8+8 bit: R/G/B/alpha in high-to-low byte order)
        const float r_n  = static_cast<float>( pc >> 24         ) / 255.0f;
        const float g_n  = static_cast<float>((pc >> 16) & 0xFFu) / 255.0f;
        const float b_n  = static_cast<float>((pc >>  8) & 0xFFu) / 255.0f;
        const float op_n = static_cast<float>( pc        & 0xFFu) / 255.0f;
        g.f_dc[0] = lerpf(chunkGet(ci, "min_r"), chunkGet(ci, "max_r"), r_n);
        g.f_dc[1] = lerpf(chunkGet(ci, "min_g"), chunkGet(ci, "max_g"), g_n);
        g.f_dc[2] = lerpf(chunkGet(ci, "min_b"), chunkGet(ci, "max_b"), b_n);
        // Compressed format stores sigmoid(opacity); invert to get raw logit
        if      (op_n <= 0.0f) g.opacity = -40.0f;
        else if (op_n >= 1.0f) g.opacity =  40.0f;
        else                   g.opacity = std::log(op_n / (1.0f - op_n));

        out.push_back(g);
    }

    return true;
}

static bool readStandard(std::istream& file,
                         const std::vector<ElementDef>& elements,
                         std::vector<GSPoint>& out)
{
    const ElementDef* vertexElem = nullptr;
    for (const auto& e : elements) {
        if (e.name == "vertex") { vertexElem = &e; break; }
    }
    if (!vertexElem) return false;

    const size_t vertexCount = vertexElem->count;

    std::unordered_map<std::string, size_t> propIndex;
    for (size_t i = 0; i < vertexElem->properties.size(); ++i)
        propIndex[vertexElem->properties[i].name] = i;

    static const char* const kRequired[] = {
        "x", "y", "z",
        "f_dc_0", "f_dc_1", "f_dc_2",
        "opacity",
        "scale_0", "scale_1", "scale_2",
        "rot_0", "rot_1", "rot_2", "rot_3"
    };
    for (const char* p : kRequired)
        if (!propIndex.count(p)) return false;

    const size_t stride = vertexElem->properties.size();
    std::vector<float> buffer(stride);
    out.reserve(vertexCount);

    auto get = [&](const char* name) -> float {
        const auto it = propIndex.find(name);
        return it != propIndex.end() ? buffer[it->second] : 0.0f;
    };

    for (size_t i = 0; i < vertexCount; ++i) {
        file.read(reinterpret_cast<char*>(buffer.data()),
                  static_cast<std::streamsize>(stride * sizeof(float)));

        GSPoint g{};
        g.x = get("x");
        g.y = get("y");
        g.z = get("z");
        if (propIndex.count("nx")) {
            g.nx = get("nx");
            g.ny = get("ny");
            g.nz = get("nz");
        }
        g.f_dc[0]  = get("f_dc_0");
        g.f_dc[1]  = get("f_dc_1");
        g.f_dc[2]  = get("f_dc_2");
        g.opacity  = get("opacity");
        g.scale[0] = get("scale_0");
        g.scale[1] = get("scale_1");
        g.scale[2] = get("scale_2");
        g.rot[0]   = get("rot_0");
        g.rot[1]   = get("rot_1");
        g.rot[2]   = get("rot_2");
        g.rot[3]   = get("rot_3");
        out.push_back(g);
    }
    return true;
}

// .splat (antimatter15-style flat binary) layout: 32 bytes per splat.
//   offset  0: float x, y, z        (position, world space)
//   offset 12: float sx, sy, sz     (already-exponentiated scale, not log)
//   offset 24: uint8 r, g, b, a     (r/g/b = sigmoid(f_dc*C0+0.5)*255, a = sigmoid(opacity)*255)
//   offset 28: uint8 rot[4]         (unit quaternion w,x,y,z, each byte = round(v*128+128))
static constexpr size_t kSplatRecordSize = 32;
static constexpr float  kShC0 = 0.28209479177387814f;

static bool readSplatFile(std::istream& file, std::vector<GSPoint>& out)
{
    file.seekg(0, std::ios::end);
    const auto fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    if (fileSize < 0 || static_cast<uint64_t>(fileSize) % kSplatRecordSize != 0)
        return false;

    const size_t count = static_cast<size_t>(fileSize) / kSplatRecordSize;
    out.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        uint8_t record[kSplatRecordSize];
        file.read(reinterpret_cast<char*>(record), static_cast<std::streamsize>(kSplatRecordSize));
        if (!file) return false;

        float pos[3];
        float scale[3];
        std::memcpy(pos, record, sizeof(pos));
        std::memcpy(scale, record + 12, sizeof(scale));
        const uint8_t* color = record + 24;
        const uint8_t* rot   = record + 28;

        GSPoint g{};
        g.x = pos[0]; g.y = pos[1]; g.z = pos[2];

        for (int k = 0; k < 3; ++k)
            g.scale[k] = std::log(std::max(scale[k], 1e-6f));

        for (int k = 0; k < 3; ++k) {
            const float n = static_cast<float>(color[k]) / 255.0f;
            g.f_dc[k] = (n - 0.5f) / kShC0;
        }

        const float a = static_cast<float>(color[3]) / 255.0f;
        if      (a <= 0.0f) g.opacity = -40.0f;
        else if (a >= 1.0f) g.opacity =  40.0f;
        else                g.opacity = std::log(a / (1.0f - a));

        for (int k = 0; k < 4; ++k)
            g.rot[k] = (static_cast<float>(rot[k]) - 128.0f) / 128.0f;

        out.push_back(g);
    }

    return true;
}

} // namespace

bool GSPointCloud::readFromFile(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file) return false;

    std::string ext = std::filesystem::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    std::vector<GSPoint> result;
    if (ext == ".splat") {
        if (!readSplatFile(file, result)) return false;
    } else {
        std::vector<ElementDef> elements;
        if (!parseHeader(file, elements) || elements.empty()) return false;

        bool compressed = false;
        for (const auto& e : elements)
            if (e.name == "chunk") { compressed = true; break; }

        if (!(compressed ? readCompressed(file, elements, result)
                         : readStandard(file, elements, result)))
            return false;
    }

    this->points = std::move(result);
    return true;
}
