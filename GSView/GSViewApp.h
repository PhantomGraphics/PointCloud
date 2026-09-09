#pragma once

#include "GSViewRenderer.h"
#include "GSViewCommandDispatcher.h"
#include "ui/GSViewPanel.h"
#include "ui/GSViewMenuBar.h"

#include "../../CGLib/VkAppBase/VkAppBase.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioRunner.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioHost.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioBrowserPanel.h"
#include "../PointCloud/GSPointCloud.h"

#include <cstddef>
#include <string>

namespace GSView {

class GSViewApp : public ::VKG::VkAppBase, public ::IScenarioHost {
public:
	GSViewApp(int width, int height, const std::string& title);

	void setInitialPLY(const std::string& path) { initialPLYPath_ = path; }

	// Scenario runner control (call before run()).
	bool loadScenario(const std::string& jsonPath) override;
	void setExitOnScenarioComplete(bool v) override { exitOnComplete_ = v; }
	int  getExitCode() const               { return exitCode_; }

	// IScenarioHost (drives ScenarioBrowserPanel)
	bool   isScenarioActive()   const override { return runner_.isActive();   }
	bool   scenarioHasFailed()  const override { return runner_.hasFailed();  }
	const std::string& scenarioFailMessage() const override { return runner_.failMessage(); }
	size_t scenarioStepCount()  const override { return runner_.stepCount();  }

	// Public PLY loader for dispatcher access.
	bool publicLoadPLY(const std::string& path, std::string& err, size_t& outCount);

protected:
	void onInit() override;
	void onUpdate(uint32_t frameIndex) override;
	void onPreRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onSwapChainCreated() override;
	void onImGui() override;
	void onCleanup() override;

private:
	Phantom::PointCloud::GSPointCloud gsCloud_;
	GSViewRenderer renderer_;
	GSViewPanel panel_;
	GSViewMenuBar menuBar_;

	GSViewCommandDispatcher dispatcher_;
	ScenarioRunner          runner_;
	ScenarioBrowserPanel    scenarioBrowser_;
	bool                    exitOnComplete_ = true;
	int                     exitCode_       = 0;

	bool loadPLY(const std::string& path, std::string& errorMessage);
	void setupCallbacks();

	std::string initialPLYPath_;
};

} // namespace GSView
