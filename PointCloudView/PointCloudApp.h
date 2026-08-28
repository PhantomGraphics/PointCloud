#pragma once

#include "../../CGLib/VkAppBase/VkAppBase.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioRunner.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioHost.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioBrowserPanel.h"

#include "PointCloudRenderer.h"
#include "SceneListPanel.h"
#include "Menu.h"
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
    Menu menuPanel_;

    CommandDispatcher dispatcher_;
    ScenarioRunner                runner_;
    ScenarioBrowserPanel          scenarioBrowser_;
    bool exitOnComplete_ = true;
    int  exitCode_       = 0;

    void syncRenderer();
    void setupCallbacks();
};

} // namespace VPC
