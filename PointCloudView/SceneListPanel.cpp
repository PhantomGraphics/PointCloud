#include "SceneListPanel.h"
#include "imgui.h"

#include <algorithm>
#include <string>

namespace VPC {

void SceneListPanel::init(World* world, int* pActiveSceneId,
                                 std::function<void()> onWorldChanged,
                                 std::function<void()> onSceneSelectionChanged)
{
    world_                    = world;
    pId_                      = pActiveSceneId;
    onWorldChanged_           = std::move(onWorldChanged);
    onSceneSelectionChanged_  = std::move(onSceneSelectionChanged);
}

void SceneListPanel::onImGui() {
    if (!world_ || !pId_) return;

    ImGui::Text("Scenes (%zu)", world_->getScenes().size());

    const float itemHeight = ImGui::GetTextLineHeightWithSpacing();
    const float listHeight = std::max(itemHeight * 4.f, 80.f);
    ImGui::BeginChild("SceneList", ImVec2(0.f, listHeight), true);

    int removeId = -1;
    for (const auto& scene : world_->getScenes()) {
        const int  id      = scene->getId();
        bool       visible = scene->isVisible();
        const bool active  = (*pId_ == id);

        ImGui::PushID(id);

        if (ImGui::Checkbox("##vis", &visible)) {
            scene->setVisible(visible);
            if (onWorldChanged_) onWorldChanged_();
        }
        ImGui::SameLine();

        const std::string label = scene->getName() +
            " (" + std::to_string(scene->getSize()) + " pts)";
        if (ImGui::Selectable(label.c_str(), active)) {
            *pId_ = id;
            if (onSceneSelectionChanged_) onSceneSelectionChanged_();
            else if (onWorldChanged_)     onWorldChanged_();
        }
        ImGui::SameLine();

        if (ImGui::SmallButton("X")) removeId = id;

        ImGui::PopID();
    }
    ImGui::EndChild();

    if (removeId >= 0) {
        const bool removedActive = (*pId_ == removeId);
        world_->removeScene(removeId);

        // Settle the active-scene id BEFORE notifying so the renderer and any
        // process panel never observe (or push) the deleted id. When the
        // active scene is the one removed, fall back to the first remaining
        // scene, or -1 if none are left.
        if (removedActive) {
            *pId_ = world_->getScenes().empty()
                        ? -1
                        : world_->getScenes().front()->getId();
        }

        if (onWorldChanged_) onWorldChanged_();
        // Signal the active-scene change too (normal-line / active-scene state),
        // not just the world rebuild.
        if (removedActive && onSceneSelectionChanged_) onSceneSelectionChanged_();
    }
}

} // namespace VPC
