#include "ProcessPanel.h"

#include "imgui.h"

namespace VPC {

void ProcessPanel::init(World* world, const int* pActiveSceneId,
                        std::function<void()> onWorldChanged,
                        std::function<void(int)> onSelectScene)
{
    world_          = world;
    pActiveSceneId_ = pActiveSceneId;
    onWorldChanged_ = std::move(onWorldChanged);
    onSelectScene_  = std::move(onSelectScene);
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

    // The page always opens; when a precondition is missing we show why and
    // grey the whole panel (its Run button included) rather than let it fail
    // silently (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 3).
    const std::string reason = blockReason();
    if (!reason.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.6f, 0.3f, 1.0f));
        ImGui::TextWrapped("Cannot run: %s", reason.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
        ImGui::Separator();
    }

    if (!world_ || !pActiveSceneId_) return;

    const bool blocked = !reason.empty();
    if (blocked) ImGui::BeginDisabled();
    view->onImGui(*world_, *pActiveSceneId_, [this](int newActiveId) {
        if (newActiveId >= 0 && onSelectScene_) onSelectScene_(newActiveId);
        if (onWorldChanged_) onWorldChanged_();
    });
    if (blocked) ImGui::EndDisabled();
}

std::string ProcessPanel::blockReason() const
{
    if (processWorksOnEmptyWorld(activeProcess_)) return {};  // Generate*

    if (!world_ || world_->isEmpty())
        return "load or generate a point cloud first.";

    const int activeId = pActiveSceneId_ ? *pActiveSceneId_ : -1;
    if (!world_->findById(activeId))
        return "select a target scene (Scenes page, or the selector in the "
               "status area above).";

    if (processNeedsReferenceScene(activeProcess_) &&
        world_->getScenes().size() < 2)
        return "this needs a second scene as the registration reference "
               "(load or generate one more).";

    return {};
}

} // namespace VPC
