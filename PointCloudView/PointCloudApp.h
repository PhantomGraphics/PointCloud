#pragma once

#include "../../CGLib/VkAppBase/VkAppBase.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioRunner.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioHost.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioBrowserPanel.h"

#include "PointCloudRenderer.h"
#include "SceneListPanel.h"
#include "ControlPage.h"
#include "ControlPanelHost.h"
#include "IEmbeddedPanel.h"
#include "PointCloudMenu.h"
#include "ProcessPanel.h"
#include "ImportExportPanel.h"
#include "CommandDispatcher.h"
#include "World.h"

#include <string>

namespace VPC {

class PointCloudApp : public ::VKG::VkAppBase, public ::IScenarioHost {
public:
    PointCloudApp(int width, int height, const std::string& title);

    bool loadPointCloudFromFile(const std::string& path, std::string& errorMessage);
    bool savePointCloudToFile (const std::string& path, std::string& errorMessage) const;

    World& getWorld() { return world_; }

    // Scenario runner control (call before run()).
    bool loadScenario(const std::string& jsonPath) override;
    void setExitOnScenarioComplete(bool v) override { exitOnComplete_ = v; }
    int  getExitCode() const               { return exitCode_; }

    // Turns off reading/writing the interactive Control-layout ini so scenario
    // and screenshot runs get a fixed, reproducible layout
    // (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 5).
    void disableInteractiveLayoutPersistence() { controlHost_.setLayoutFile({}); }

    // IScenarioHost (drives ScenarioBrowserPanel)
    bool   isScenarioActive()   const override { return runner_.isActive();   }
    bool   scenarioHasFailed()  const override { return runner_.hasFailed();  }
    const std::string& scenarioFailMessage() const override { return runner_.failMessage(); }
    size_t scenarioStepCount()  const override { return runner_.stepCount();  }

protected:
    void onInit()             override;
    void onSwapChainCreated() override;
    void onUpdate(uint32_t frameIndex) override;
    void onImGui()            override;
    void onCleanup()          override;

private:
    World world_;

    PointCloudRenderer  renderer_;
    SceneListPanel      sceneListPanel_;

    // The single shared "Control" window and the panels it embeds.
    ControlPanelHost  controlHost_;
    PointCloudMenu    menu_;
    ProcessPanel      processPanel_;
    ImportExportPanel importExportPanel_;
    FnEmbeddedPanel   scenesEmbed_;
    FnEmbeddedPanel   renderingEmbed_;
    FnEmbeddedPanel   scenarioBrowserEmbed_;

    CommandDispatcher             dispatcher_;
    ScenarioRunner                runner_;
    ScenarioBrowserPanel          scenarioBrowser_;
    bool exitOnComplete_ = true;
    int  exitCode_       = 0;

    void syncRenderer();
    void setupCallbacks();
    void registerControlPages();
    void drawStatusArea();
    void drawMenuBar();
};

} // namespace VPC
