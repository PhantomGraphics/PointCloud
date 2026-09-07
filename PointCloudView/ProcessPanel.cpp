#include "ProcessPanel.h"

#include "imgui.h"

namespace VPC {

void ProcessPanel::init(World* world, const int* pActiveSceneId,
                        std::function<void()> onWorldChanged)
{
    world_          = world;
    pActiveSceneId_ = pActiveSceneId;
    onWorldChanged_ = std::move(onWorldChanged);
}

void ProcessPanel::setProcess(ProcessId id)
{
    activeProcess_ = id;
}

IProcessView* ProcessPanel::viewFor(ProcessId id)
{
    const int idx = static_cast<int>(id);
    if (idx < 0 || idx >= kProcessCount) return nullptr;
    if (!views_[idx]) views_[idx] = makeProcessView(id);
    return views_[idx].get();
}

void ProcessPanel::resetActiveProcess()
{
    const int idx = static_cast<int>(activeProcess_);
    if (idx >= 0 && idx < kProcessCount) views_[idx].reset();
}

void ProcessPanel::drawContents()
{
    if (activeProcess_ == ProcessId::None) {
        ImGui::TextWrapped(
            "Pick an operation from the PointCloud menu "
            "(Generate / Features / Filters / Segmentation / Fitting / "
            "Registration / Surface).");
        return;
    }

    IProcessView* view = viewFor(activeProcess_);
    if (!view) {
        ImGui::TextWrapped("This operation is not available.");
        return;
    }

    ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "[%s]", view->getName());
    ImGui::SameLine();
    if (ImGui::SmallButton("Reset")) {
        resetActiveProcess();
        view = viewFor(activeProcess_);
    }
    ImGui::Spacing();

    if (world_ && pActiveSceneId_) {
        view->onImGui(*world_, *pActiveSceneId_,
                      [this]() { if (onWorldChanged_) onWorldChanged_(); });
    }
}

} // namespace VPC
