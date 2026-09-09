#include "GSViewCommandDispatcher.h"
#include "GSViewApp.h"
#include "GSViewRenderer.h"
#include "GaussianPointRenderer.h"

#include <charconv>
#include <cstdio>
#include <filesystem>
#include <fstream>
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
    if (name == "GetGpAccumFrames")       return cmdGetGpAccumFrames();
    if (name == "GetGpProfile")           return cmdGetGpProfile();
    if (name == "GetGpTimings")           return cmdGetGpTimings();
    if (name == "GetGpPointBudget")       return cmdGetGpPointBudget();
    if (name == "ValidateGpOracle")       return cmdValidateGpOracle();
    if (name == "GetGpShDegree")          return cmdGetGpShDegree();
    if (name == "GetGpTonemap")           return cmdGetGpTonemap();
    if (name == "GetGpGamma")             return cmdGetGpGamma();
    if (name == "GetPbvr3dMethod")        return cmdGetPbvr3dMethod();

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
    if (name == "SetGpShDegree")          return cmdSetGpShDegree(rest);
    if (name == "SetGpTonemap")           return cmdSetGpTonemap(rest);
    if (name == "SetGpGamma")             return cmdSetGpGamma(rest);
    if (name == "SetPbvr3dMethod")        return cmdSetPbvr3dMethod(rest);
    if (name == "SetGpPointBudget")       return cmdSetGpPointBudget(rest);
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
        if (!renderer_->isGaussianPointAvailable())
            return "Error:PBVR3DExperimental unavailable (renderer init failed)";
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

// ---- PBVR3DExperimental (Phase 4) ---------------------------------------------

std::string GSViewCommandDispatcher::cmdSetPbvr3dMethod(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    int m;
    if      (arg == "proportional")     m = 0;
    else if (arg == "extinction")       m = 1;
    else if (arg == "view_conditioned") m = 2;
    else if (!tryInt(arg, m) || m < 0 || m > 2)
        return "Error:method must be proportional|extinction|view_conditioned";
    renderer_->setPbvr3dMethod(m);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdGetPbvr3dMethod()
{
    if (!renderer_) return "Val:proportional";
    switch (renderer_->getPbvr3dMethod()) {
        case 1:  return "Val:extinction";
        case 2:  return "Val:view_conditioned";
        default: return "Val:proportional";
    }
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
           ",Drawn:" + std::to_string(s.drawnPoints) +
           ",Candidates:" + std::to_string(s.candidateCount) +
           ",Accum:" + std::to_string(s.accumFrames) +
           ",ShDeg:" + std::to_string(s.shDegreeData);
}

std::string GSViewCommandDispatcher::cmdGetGpAccumFrames()
{
    if (!renderer_) return "Val:0";
    return "Val:" + std::to_string(renderer_->getGaussianPointStats().accumFrames);
}

std::string GSViewCommandDispatcher::buildProfile() const
{
    if (!renderer_) return "";
    const auto& p = renderer_->getGaussianPointParams();
    const auto  s = renderer_->getGaussianPointStats();
    const VkExtent2D ext = renderer_->getExtent();

    std::string path = "GaussianPoint";
    std::string method = "-";
    switch (renderer_->getRenderMode()) {
        case RenderMode::PBVR3DExperimental:
            path = "PBVR3D";
            method = renderer_->getPbvr3dMethod() == 1 ? "extinction"
                   : renderer_->getPbvr3dMethod() == 2 ? "view_conditioned" : "proportional";
            break;
        case RenderMode::SortBased: path = "SortBased"; break;
        default: break;
    }

    char buf[768];
    std::snprintf(buf, sizeof(buf),
        "gpu=%s;path=%s;method=%s;res=%ux%u;spp=%d;seed=%s;densityScale=%s;pointBudget=%s;"
        "shDegree=%d;shDegreeData=%d;tonemap=%d;gamma=%s;expected=%u;generated=%u;activeSamples=%u;"
        "drawnPoints=%u;candidatePoints=%u;accumFrames=%u;computeMs=%.3f;clearMs=%.3f;depthMs=%.3f;colorMs=%.3f;resolveMs=%.3f",
        renderer_->getGaussianPointGpuName().c_str(), path.c_str(), method.c_str(),
        ext.width, ext.height, p.sppSide * p.sppSide,
        p.seedMode == 0 ? "deterministic" : "frame",
        fmtF(p.densityScale).c_str(), fmtF(p.pointBudget).c_str(), p.shDegree, s.shDegreeData,
        p.tonemapMode, fmtF(p.gamma).c_str(),
        s.expectedCount, s.generatedCount, s.activeSamples, s.drawnPoints, s.candidateCount, s.accumFrames,
        s.computeMs, s.clearMs, s.splatDepthMs, s.splatColorMs, s.resolveMs);
    return buf;
}

std::string GSViewCommandDispatcher::cmdGetGpProfile()
{
    const std::string prof = buildProfile();
    return prof.empty() ? "Error:renderer not available" : ("Profile:" + prof);
}

std::string GSViewCommandDispatcher::cmdGetGpTimings()
{
    if (!renderer_) return "Error:renderer not available";
    const auto s = renderer_->getGaussianPointStats();
    char buf[192];
    std::snprintf(buf, sizeof(buf),
        "Compute:%.3f,Clear:%.3f,Depth:%.3f,Color:%.3f,Resolve:%.3f",
        s.computeMs, s.clearMs, s.splatDepthMs, s.splatColorMs, s.resolveMs);
    return buf;
}

std::string GSViewCommandDispatcher::cmdGetGpPointBudget()
{
    if (!renderer_) return "Val:0";
    return "Val:" + fmtF(renderer_->getGaussianPointParams().pointBudget);
}

std::string GSViewCommandDispatcher::cmdValidateGpOracle()
{
    if (!renderer_) return "Error:renderer not available";
    double all = 0.0, foreground = 0.0;
    size_t foregroundSamples = 0;
    if (!renderer_->validateGaussianPointOracle(all, foreground, foregroundSamples))
        return "Error:validation requires GaussianPoint, density=1, budget=0, and a visible cloud";
    char buf[160];
    std::snprintf(buf, sizeof(buf), "PSNR:%.3f,ForegroundPSNR:%.3f,Samples:%zu",
                  all, foreground, foregroundSamples);
    // Overall PSNR is the primary plan gate. The foreground-only floor catches
    // gross coordinate/colour errors without making 34 stochastic sets flaky.
    if (all < 25.0 || foreground < 18.0) return std::string("Error:") + buf;
    return std::string("OK:") + buf;
}

std::string GSViewCommandDispatcher::cmdSetGpPointBudget(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    if (v < 0.0f) return "Error:budget must be >= 0";
    auto p = renderer_->getGaussianPointParams();
    p.pointBudget = v;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdGetGpShDegree()
{
    if (!renderer_) return "Val:0";
    return "Val:" + std::to_string(renderer_->getGaussianPointParams().shDegree);
}

std::string GSViewCommandDispatcher::cmdGetGpTonemap()
{
    if (!renderer_) return "Val:none";
    switch (renderer_->getGaussianPointParams().tonemapMode) {
        case 1:  return "Val:reinhard";
        case 2:  return "Val:aces";
        default: return "Val:none";
    }
}

std::string GSViewCommandDispatcher::cmdGetGpGamma()
{
    if (!renderer_) return "Val:1";
    return "Val:" + fmtF(renderer_->getGaussianPointParams().gamma);
}

std::string GSViewCommandDispatcher::cmdSetGpShDegree(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    int d;
    if (!tryInt(arg, d)) return "Error:invalid int";
    if (d < 0 || d > 3) return "Error:SH degree must be 0..3";
    auto p = renderer_->getGaussianPointParams();
    p.shDegree = d;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetGpTonemap(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    auto p = renderer_->getGaussianPointParams();
    if      (arg == "none")     p.tonemapMode = 0;
    else if (arg == "reinhard") p.tonemapMode = 1;
    else if (arg == "aces")     p.tonemapMode = 2;
    else return "Error:unknown tonemap " + arg;
    renderer_->setGaussianPointParams(p);
    return "OK";
}

std::string GSViewCommandDispatcher::cmdSetGpGamma(const std::string& arg)
{
    if (!renderer_) return "Error:renderer not available";
    float v;
    if (!tryFloat(arg, v)) return "Error:invalid float";
    if (v < 0.1f || v > 4.0f) return "Error:gamma must be 0.1..4.0";
    auto p = renderer_->getGaussianPointParams();
    p.gamma = v;
    renderer_->setGaussianPointParams(p);
    return "OK";
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

    // Sidecar metadata (Phase 6): <image>.json with the render profile.
    const std::string prof = buildProfile();
    if (!prof.empty()) {
        std::ofstream sc(abs + ".json");
        if (sc) {
            sc << "{\n";
            bool first = true;
            size_t i = 0;
            while (i < prof.size()) {
                const size_t sep = prof.find(';', i);
                const std::string kv = prof.substr(i, sep == std::string::npos ? std::string::npos : sep - i);
                const size_t eq = kv.find('=');
                if (eq != std::string::npos) {
                    if (!first) sc << ",\n";
                    sc << "  \"" << kv.substr(0, eq) << "\": \"" << kv.substr(eq + 1) << "\"";
                    first = false;
                }
                if (sep == std::string::npos) break;
                i = sep + 1;
            }
            sc << "\n}\n";
        }
    }
    return "OK:" + abs;
}

} // namespace GSView
