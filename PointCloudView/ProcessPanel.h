#pragma once

#include "IEmbeddedPanel.h"
#include "ProcessRegistry.h"
#include "World.h"

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace VPC {

// The "Processing" page: shows one algorithm panel at a time, selected from the
// PointCloud menu. Panels are created lazily on first selection and kept for the
// rest of the session so their parameters survive page/process switches
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 4). Panels hold no
// raw pointer to a scene and no bulk point copy -- each Run resolves its target
// from the active scene id.
class ProcessPanel : public IEmbeddedPanel {
public:
    void init(World* world, const int* pActiveSceneId,
              std::function<void()> onWorldChanged);

    void setProcess(ProcessId id);
    ProcessId getProcess() const { return activeProcess_; }

    // Discards the retained panel for the active process so its parameters go
    // back to their defaults on next use (Phase 2).
    void resetActiveProcess();

    void drawContents() override;

private:
    IProcessView* viewFor(ProcessId id);

    // Non-empty = the active process cannot run yet; the string says why.
    std::string blockReason() const;

    World*     world_          = nullptr;
    const int* pActiveSceneId_ = nullptr;
    std::function<void()> onWorldChanged_;

    ProcessId activeProcess_ = ProcessId::None;
    std::array<std::unique_ptr<IProcessView>, kProcessCount> views_{};
};

} // namespace VPC
