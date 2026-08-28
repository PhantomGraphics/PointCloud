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

    // Called after any command that modifies the world; arg is the new active scene id.
    void setOnWorldChanged(std::function<void(int)> cb) { onWorldChanged_ = std::move(cb); }

    // Call from the render thread (onUpdate) every frame.
    void processQueue();

    // IScenarioDispatcher
    void dispatch(const std::string& command) override;
    std::vector<std::string> collectResponses() override;

private:
    std::string route(const std::string& cmd);

    VPC::World*              world_    = nullptr;
    std::function<void(int)> onWorldChanged_;
    int                      activeId_ = -1;

    // Scalar results from the most recent processing command that don't fit the
    // "new scene of points" model (fitness, inlier counts, hull area, ...).
    // Read back via "GetLastMetric:<name>". See docs/todo/PLAN_pointcloudview_new_algorithm_views.md §3.3.
    std::map<std::string, std::string> lastMetrics_;

    std::mutex              mutex_;
    std::queue<std::string> inputQueue_;
    std::queue<std::string> outputQueue_;
};
