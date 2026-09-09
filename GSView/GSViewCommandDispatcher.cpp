#include "GSViewCommandDispatcher.h"
#include "GSViewApp.h"
#include "GSViewRenderer.h"
#include "GaussianPointRenderer.h"

#include <charconv>
#include <cstdio>
#include <filesystem>
#include <string>

namespace GSView {

// ---- parse helpers ----------------------------------------------------------

static bool tryFloat(const std::string& s, float& out)
{
    const char* b = s.data();
    const char* e = b + s.size();
    const auto [ptr, ec] = std::from_chars(b, e, out);
    return ec == std::errc{} && ptr == e;
}

static bool tryInt(const std::string& s, int& out)
{
    const char* b = s.data();
    const char* e = b + s.size();
    const auto [ptr, ec] = std::from_chars(b, e, out);
    return ec == std::errc{} && ptr == e;
}

// "%.6g" formatting: strips trailing zeros (10.0 -> "10", 1.5 -> "1.5")
static std::string fmtF(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

// ---- public interface -------------------------------------------------------

void GSViewCommandDispatcher::enqueue(const std::string& cmd)
{
    std::lock_guard<std::mutex> lk(mutex_);
    inputQueue_.push(cmd);
}

void GSViewCommandDispatcher::processQueue()
{
    std::queue<std::string> local;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        std::swap(local, inputQueue_);
    }
    while (!local.empty()) {
        const std::string resp = route(local.front());
        local.pop();
        {
            std::lock_guard<std::mutex> lk(mutex_);
            outputQueue_.push(resp);
        }
    }
}

std::vector<std::string> GSViewCommandDispatcher::drainResponses()
{
    std::vector<std::string> out;
    std::lock_guard<std::mutex> lk(mutex_);
    while (!outputQueue_.empty()) {
        out.push_back(std::move(outputQueue_.front()));
        outputQueue_.pop();
    }
    return out;
}

// ---- route ------------------------------------------------------------------

std::string GSViewCommandDispatcher::route(const std::string& cmd)
{
    const auto c0   = cmd.find(':');
    const auto name = cmd.substr(0, c0);
    const auto rest = (c0 != std::string::npos) ? cmd.substr(c0 + 1) : std::string{};

    // --- no-arg commands ---
    if (name == "GetStatus")              return cmdGetStatus();
    if (name == "GetSplatCount")          return cmdGetSplatCount();
    if (name == "GetParticleCount")       return cmdGetParticleCount();
    if (name == "GetParticleCapacity")    return cmdGetParticleCapacity();
    if (name == "GetDataGeneration")      return cmdGetDataGeneration();
    if (name == "GetRenderMode")          return cmdGetRenderMode();
    if (name == "GetSplatSizeScale")      return cmdGetSplatSizeScale();
    if (name == "GetSortPointSize")       return cmdGetSortPointSize();
    if (name == "GetDensityScale")        return cmdGetDensityScale();
    if (name == "GetMaxParticlesPerSplat")return cmdGetMaxParticlesPerSplat();
    if (name == "GetPbvrParticleSize")    return cmdGetPbvrParticleSize();
    if (name == "GetGSAvailable")         return cmdGetGSAvailable();
    if (name == "GetGaussianPointAvailable") return cmdGetGaussianPointAvailable();
    if (name == "GetGpSpp")               return cmdGetGpSpp();
    if (name == "GetGpSeedMode")          return cmdGetGpSeedMode();
    if (name == "GetGpDensityScale")      return cmdGetGpDensityScale();
    if (name == "GetGpGeneratedCount")    return cmdGetGpGeneratedCount();
    if (name == "GetGpStats")             return cmdGetGpStats();

    if (rest.empty()) return "Error:missing argument for " + name;

    // --- commands with arguments ---
    if (name == "SetRenderMode")          return cmdSetRenderMode(rest);
    if (name == "SetSplatSizeScale")      return cmdSetSplatSizeScale(rest);
    if (name == "SetSortPointSize")       return cmdSetSortPointSize(rest);
    if (name == "SetDensityScale")        return cmdSetDensityScale(rest);
    if (name == "SetMaxParticlesPerSplat")return cmdSetMaxParticlesPerSplat(rest);
    if (name == "SetPbvrParticleSize")    return cmdSetPbvrParticleSize(rest);
    if (name == "SetGpSpp")               return cmdSetGpSpp(rest);
    if (name == "SetGpSeedMode")          return cmdSetGpSeedMode(rest);
    if (name == "SetGpDensityScale")      return cmdSetGpDensityScale(rest);
    if (name == "LoadPLY")                return cmdLoadPLY(rest);
    if (name == "Screenshot")             return cmdScreenshot(rest);

    return "Error:unknown command " + name;
}

// ---- query commands ---------------------------------------------------------

std::string GSViewCommandDispatcher::cmdGetStatus()
{
    return "OK";
}

std::string GSViewCommandDispatcher::cmdGetSplatCount()
{
    if (!renderer_) return "Count:0";
    return "Count:" + std::to_string(renderer_->getSplatCount());
}

std::string GSViewCommandDispatcher::cmdGetParticleCount()
{
    if (!renderer_) return "Count:0";
    return "Count:" + std::to_string(renderer_->getParticleCount());
}

std::string GSViewCommandDispatcher::cmdGetParticleCapacity()
{
    if (!renderer_) return "Count:0";
    return "Count:" + std::to_string(renderer_->getParticleCapacity());
}

std::string GSViewCommandDispatcher::cmdGetDataGeneration()
{
    if (!renderer_) return "Val:0";
    return "Val:" + std::to_string(renderer_->getDataGeneration());
}

std::string GSViewCommandDispatcher::cmdGetRenderMode()
{
    if (!renderer_) return "Val:SortBased";
    switch (renderer_->getRenderMode()) {
        case RenderMode::PBVR3DExperimental: return "Val:PBVR3DExperimental";
        case RenderMode::GaussianPoint:      return "Val:GaussianPoint";
        default:                             return "Val:SortBased";
    }
}

std::string GSViewCommandDispatcher::cmdGetSplatSizeScale()
{
    if (!renderer_) return "Val:" + fmtF(1000.f);
    return "Val:" + fmtF(renderer_->getSplatSizeScale());
}

std::string GSViewCommandDispatcher::cmdGetSortPointSize()
{
    return "Val:" + fmtF(lastSortPointSize_);
}

std::string GSViewCommandDispatcher::cmdGetDensityScale()
{
    return "Val:" + fmtF(lastDensityScale_);
}

std::string GSViewCommandDispatcher::cmdGetMaxParticlesPerSplat()
{
    return "Val:" + std::to_string(lastMaxParticles_);
}

std::string GSViewCommandDispatcher::cmdGetPbvrParticleSize()
{
    return "Val:" + fmtF(lastPbvrParticleSize_);
}

std::string GSViewCommandDispatcher::cmdGetGSAvailable()
{
    if (!renderer_) return "Val:0";
    return renderer_->isGSAvailable() ? "Val:1" : "Val:0";
}

// ---- mutation commands ------------------------------------------------------

std::string GSViewCommandDispatcher::cmdSetRenderMode(const std::string& mode)
{
    if (!renderer_) return "Error:renderer not available";
    if (mode == "SortBased") {
        renderer_->setRenderMode(RenderMode::SortBased);
        return "OK";
    }
    // "PBVR" is a temporary backward-compat alias for the renamed experimental mode
    // (Phase 0). Old scenario files keep working; new ones use the canonical name.
    if (mode == "PBVR3DExperimental" || mode == "PBVR") {
        renderer_->setRenderMode(RenderMode::PBVR3DExperimental);
        return "OK";
    }
    if (mode == "GaussianPoint") {
        if (!renderer_->isGaussianPointAvailable())
            return "Error:GaussianPoint unavailable (renderer init failed)";
        renderer_->setRenderMode(RenderMode::GaussianPoint);
        return "OK";
    }
    return "Error:unknown mode " + mode;
}

std::string GSViewCommandDispatcher::cmdSetSplatSizeScale(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    renderer_->setSplatSizeScale(v);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetSortPointSize(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    renderer_->setSortPointSize(v);
    lastSortPointSize_ = v;
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetDensityScale(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    renderer_->setDensityScale(v);
    lastDensityScale_ = v;
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetMaxParticlesPerSplat(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    int n;
    if (!tryInt(arg, n)) return "Error:invalid int";
    renderer_->setMaxParticlesPerSplat(n);
    lastMaxParticles_ = n;
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetPbvrParticleSize(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    renderer_->setPbvrParticleSize(v);
    lastPbvrParticleSize_ = v;
    return "OK";
}

// ---- GaussianPoint (Phase 2) ----------------------------------------------

std::string GSViewCommandDispatcher::cmdGetGaussianPointAvailable()
{
    if (!renderer_) return "Val:0";
    return renderer_->isGaussianPointAvailable() ? "Val:1" : "Val:0";
}

std::string GSViewCommandDispatcher::cmdGetGpSpp()
{
    if (!renderer_) return "Val:0";
    const int side = renderer_->getGaussianPointParams().sppSide;
    return "Val:" + std::to_string(side * side);
}

std::string GSViewCommandDispatcher::cmdGetGpSeedMode()
{
    if (!renderer_) return "Val:frame";
    return renderer_->getGaussianPointParams().seedMode == 0 ? "Val:deterministic" : "Val:frame";
}

std::string GSViewCommandDispatcher::cmdGetGpDensityScale()
{
    if (!renderer_) return "Val:1";
    return "Val:" + fmtF(renderer_->getGaussianPointParams().densityScale);
}

std::string GSViewCommandDispatcher::cmdGetGpGeneratedCount()
{
    if (!renderer_) return "Count:0";
    return "Count:" + std::to_string(renderer_->getGaussianPointStats().generatedCount);
}

std::string GSViewCommandDispatcher::cmdGetGpStats()
{
    if (!renderer_) return "Error:renderer not available";
    const auto s = renderer_->getGaussianPointStats();
    return "Expected:" + std::to_string(s.expectedCount) +
           ",Generated:" + std::to_string(s.generatedCount) +
           ",Active:" + std::to_string(s.activeSamples) +
           ",Drawn:" + std::to_string(s.drawnPoints);
}

std::string GSViewCommandDispatcher::cmdSetGpSpp(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    int spp;
    if (!tryInt(arg, spp)) return "Error:invalid int";
    int side = 1;
    if      (spp >= 16) side = 4;
    else if (spp >= 9)  side = 3;
    else if (spp >= 4)  side = 2;
    auto p = renderer_->getGaussianPointParams();
    p.sppSide = side;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetGpSeedMode(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    auto p = renderer_->getGaussianPointParams();
    if      (arg == "deterministic") p.seedMode = 0;
    else if (arg == "frame")         p.seedMode = 1;
    else return "Error:unknown seed mode " + arg;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetGpDensityScale(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    if (v <= 0.0f) return "Error:density scale must be positive";
    auto p = renderer_->getGaussianPointParams();
    p.densityScale = v;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

// ---- file / IO commands -----------------------------------------------------

std::string GSViewCommandDispatcher::cmdLoadPLY(const std::string& path)
{
    if (!app_) return "Error:app not available";
    std::string err;
    size_t count = 0;
    if (!app_->publicLoadPLY(path, err, count))
        return "Error:" + err;
    return "OK:" + std::to_string(count);
}

std::string GSViewCommandDispatcher::cmdScreenshot(const std::string& path)
{
    if (path.empty()) return "Error:path is empty";
    if (!app_)        return "Error:app not available";
    const std::string abs = std::filesystem::absolute(path).string();
    std::filesystem::create_directories(std::filesystem::path(abs).parent_path());
    app_->requestScreenshot(abs);
    return "OK:" + abs;
}

} // namespace GSView
