#pragma once

#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioDispatcher.h"

#include <mutex>
#include <queue>
#include <string>
#include <vector>

namespace GSView {

class GSViewApp;
class GSViewRenderer;

class GSViewCommandDispatcher : public IScenarioDispatcher {
public:
    void setApp(GSViewApp* a)           { app_      = a; }
    void setRenderer(GSViewRenderer* r) { renderer_ = r; }

    // Thread-safe: may be called from any thread.
    void enqueue(const std::string& cmd);

    // Call from the render thread (onUpdate) every frame.
    void processQueue();

    // Drain all pending responses.
    std::vector<std::string> drainResponses();

    // IScenarioDispatcher
    void dispatch(const std::string& cmd) override { enqueue(cmd); }
    std::vector<std::string> collectResponses() override { return drainResponses(); }

private:
    std::string route(const std::string& cmd);

    // Query commands
    std::string cmdGetStatus();
    std::string cmdGetSplatCount();
    std::string cmdGetParticleCount();
    std::string cmdGetParticleCapacity();
    std::string cmdGetDataGeneration();
    std::string cmdGetRenderMode();
    std::string cmdGetSplatSizeScale();
    std::string cmdGetSortPointSize();
    std::string cmdGetDensityScale();
    std::string cmdGetMaxParticlesPerSplat();
    std::string cmdGetPbvrParticleSize();
    std::string cmdGetGSAvailable();

    // GaussianPoint (Phase 2)
    std::string cmdGetGaussianPointAvailable();
    std::string cmdGetGpSpp();
    std::string cmdGetGpSeedMode();
    std::string cmdGetGpDensityScale();
    std::string cmdGetGpGeneratedCount();
    std::string cmdGetGpStats();
    std::string cmdGetGpAccumFrames();
    std::string cmdGetGpShDegree();
    std::string cmdGetGpTonemap();
    std::string cmdGetGpGamma();
    std::string cmdSetGpSpp(const std::string& arg);
    std::string cmdSetGpSeedMode(const std::string& arg);
    std::string cmdSetGpDensityScale(const std::string& arg);
    std::string cmdSetGpShDegree(const std::string& arg);
    std::string cmdSetGpTonemap(const std::string& arg);
    std::string cmdSetGpGamma(const std::string& arg);

    // Mutation commands
    std::string cmdSetRenderMode(const std::string& mode);
    std::string cmdSetSplatSizeScale(const std::string& arg);
    std::string cmdSetSortPointSize(const std::string& arg);
    std::string cmdSetDensityScale(const std::string& arg);
    std::string cmdSetMaxParticlesPerSplat(const std::string& arg);
    std::string cmdSetPbvrParticleSize(const std::string& arg);
    std::string cmdSetPbvr3dMethod(const std::string& arg);
    std::string cmdGetPbvr3dMethod();

    // File / IO commands
    std::string cmdLoadPLY(const std::string& path);
    std::string cmdScreenshot(const std::string& path);

    // Cached parameter values (renderer exposes no getters for these).
    // Defaults match GSViewPanel defaults.
    float lastSortPointSize_    = 2.0f;
    float lastDensityScale_     = 1.0f;
    int   lastMaxParticles_     = 2048;
    float lastPbvrParticleSize_ = 8.0f;

    GSViewApp*      app_      = nullptr;
    GSViewRenderer* renderer_ = nullptr;

    std::mutex              mutex_;
    std::queue<std::string> inputQueue_;
    std::queue<std::string> outputQueue_;
};

} // namespace GSView
