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

	// Headless single-shot capture (call before run()): initial render mode
	// (see GSView::parseRenderModeName for accepted names), initial camera
	// pose for the GaussianPoint/PBVR3D evaluation camera (see
	// GSViewRenderer::setEvaluationCamera), and whether to close the window
	// (causing run() to return) right after the requested --screenshot is
	// written -- turning --screenshot into a true one-shot headless capture.
	void setInitialRenderMode(const std::string& mode) { initialRenderMode_ = mode; }
	void setInitialCamera(float theta, float phi, float distance) {
		hasInitialCamera_ = true;
		initialCamTheta_ = theta;
		initialCamPhi_ = phi;
		initialCamDistance_ = distance;
	}
	void setExitAfterScreenshot(bool v) { exitAfterScreenshot_ = v; }
	void setInitialCameraFlipY(bool v) { initialCameraFlipY_ = v; }
	// Suppresses all ImGui drawing (menu bar, control panel, debug overlays) so
	// --screenshot captures a plain render of the scene, for use as a general
	// offscreen renderer rather than an app-state verification snapshot.
	void setHideUI(bool v) { hideUI_ = v; }

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
	std::string initialRenderMode_;
	bool        hasInitialCamera_    = false;
	bool        initialCameraFlipY_  = false;
	float       initialCamTheta_     = 0.f;
	float       initialCamPhi_       = 0.f;
	float       initialCamDistance_  = 0.f;
	bool        exitAfterScreenshot_ = false;
	bool        hideUI_               = false;
};

} // namespace GSView
