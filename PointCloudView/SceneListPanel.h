#pragma once
#include "World.h"
#include <functional>

namespace VPC {

// Renders the scene list section inside an existing ImGui window.
// Call onImGui() from within ImGui::Begin("Control") / ImGui::End().
class SceneListPanel {
public:
    void init(World* world, int* pActiveSceneId,
              std::function<void()> onWorldChanged,
              std::function<void()> onSceneSelectionChanged = {});

    void onImGui();

private:
    World* world_  = nullptr;
    int*   pId_    = nullptr;
    std::function<void()> onWorldChanged_;
    std::function<void()> onSceneSelectionChanged_;
};

} // namespace VPC
