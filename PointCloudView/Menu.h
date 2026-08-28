#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "World.h"
#include "IProcessView.h"
#include "PointCloudRenderer.h"

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace VPC {

class SceneListPanel;

// Manages the "Control" ImGui window, the PointCloud menu bar, and the active
// process-view panel.
//
// Usage:
//   menuPanel_.onImGuiMenuBar()  窶・call inside BeginMainMenuBar() context
//   menuPanel_.onImGui()         窶・IVkUIPanel hook; creates the "Control" window
class Menu : public ::VKG::IVkUIPanel {
public:
    void init(World* world, int* pActiveSceneId,
              std::function<void()> onWorldChanged,
              PointCloudRenderer::RenderMode* pRenderMode,
              SceneListPanel* scenePanel,
              PointCloudRenderer* renderer,
              std::function<bool(const std::string&, std::string&)> onLoad,
              std::function<bool(const std::string&, std::string&)> onSave);

    void onImGuiMenuBar();
    void onImGui() override;  // IVkUIPanel: creates and owns the "Control" window

private:
    World* world_   = nullptr;
    int*   pId_     = nullptr;
    PointCloudRenderer::RenderMode* pRenderMode_ = nullptr;
    std::function<void()> onWorldChanged_;
    std::unique_ptr<IProcessView> activeProcessView_;

    SceneListPanel*     scenePanel_ = nullptr;
    PointCloudRenderer* renderer_   = nullptr;

    // Import/Export state (moved from PointCloudApp)
    std::array<char, 512> importPath_{};
    std::array<char, 512> exportPath_{};
    std::string statusMessage_;
    std::function<bool(const std::string&, std::string&)> onLoad_;
    std::function<bool(const std::string&, std::string&)> onSave_;

    void drawProcessView();
    void drawImportExport();
};

} // namespace VPC
