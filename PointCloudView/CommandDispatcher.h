#pragma once
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioDispatcher.h"
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

namespace VPC { class World; }

class CommandDispatcher : public IScenarioDispatcher {
public:
    void setWorld(VPC::World* w) { world_ = w; }

    // Share the GUI's active-scene id (renderer_.getActiveSceneIdPtr()) so a
    // scenario acts on whatever the GUI last selected and vice versa -- the two
    // entry points no longer keep separate "current scene" state
    // (docs/todo/PLAN_pointcloudview_gui_restructuring.md Phase 4).
    void setActiveSceneIdRef(int* p) { pActiveSceneId_ = p; }

    // Called after any command that modifies the world; arg is the new active scene id.
    void setOnWorldChanged(std::function<void(int)> cb) { onWorldChanged_ = std::move(cb); }

    // Call from the render thread (onUpdate) every frame.
    void processQueue();

    // IScenarioDispatcher
    void dispatch(const std::string& command) override;
    std::vector<std::string> collectResponses() override;

private:
    std::string route(const std::string& cmd);

    // The active scene id, shared with the renderer/GUI when setActiveSceneIdRef()
    // was called (always is, in PointCloudApp); a private fallback otherwise
    // (e.g. a unit test that constructs a bare dispatcher).
    int& activeId() { return pActiveSceneId_ ? *pActiveSceneId_ : activeIdFallback_; }

    VPC::World*              world_    = nullptr;
    std::function<void(int)> onWorldChanged_;
    int*                     pActiveSceneId_   = nullptr;
    int                      activeIdFallback_ = -1;

    // Scalar results from the most recent processing command that don't fit the
    // "new scene of points" model (fitness, inlier counts, hull area, ...).
    // Read back via "GetLastMetric:<name>".
    std::map<std::string, std::string> lastMetrics_;

    std::mutex              mutex_;
    std::queue<std::string> inputQueue_;
    std::queue<std::string> outputQueue_;
};
