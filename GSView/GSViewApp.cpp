#include "GSViewApp.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <string>
#include <vector>

namespace GSView {

namespace {
// Shortest text that reads back as the same float (commands parse with from_chars).
std::string fmtArg(float v)
{
	char buf[32];
	std::snprintf(buf, sizeof(buf), "%.9g", v);
	return buf;
}

// Commands equivalent to the GUI having changed GP params `from` -> `to`; every
// field the GUI edits has a command, so no lambda path is needed.
std::vector<std::string> gpParamCommands(const GaussianPointRenderer::Params& from,
                                         const GaussianPointRenderer::Params& to)
{
	std::vector<std::string> c;
	const auto flag = [](bool b) { return std::string(b ? "1" : "0"); };
	if (to.lodMode != from.lodMode)
		c.push_back(std::string("SetGpLodMode:") + (to.lodMode == 1 ? "manual" : to.lodMode == 2 ? "adaptive" : "off"));
	if (to.ensemblesPerFrame != from.ensemblesPerFrame) c.push_back("SetGpEnsemblesPerFrame:" + std::to_string(to.ensemblesPerFrame));
	if (to.targetEnsembles != from.targetEnsembles) c.push_back("SetGpTargetEnsembles:" + std::to_string(to.targetEnsembles));
	if (to.lodFrameBudgetLowMs != from.lodFrameBudgetLowMs || to.lodFrameBudgetHighMs != from.lodFrameBudgetHighMs)
		c.push_back("SetGpLodFrameBudget:" + fmtArg(to.lodFrameBudgetLowMs) + "," + fmtArg(to.lodFrameBudgetHighMs));
	if (to.pbvrZoomRecalibration != from.pbvrZoomRecalibration) c.push_back("SetPbvrZoom:" + flag(to.pbvrZoomRecalibration));
	if (to.pbvrReferencePixelLength != from.pbvrReferencePixelLength)
		c.push_back("SetPbvrReferencePixelLength:" + fmtArg(to.pbvrReferencePixelLength));
	if (to.pbvrFootprintCalibration != from.pbvrFootprintCalibration)
		c.push_back("SetPbvrFootprintCalibration:" + std::to_string(to.pbvrFootprintCalibration));
	if (to.pbvrRadialCorrection != from.pbvrRadialCorrection) c.push_back("SetPbvrRadialCorrection:" + flag(to.pbvrRadialCorrection));
	if (to.pbvrCentreDepth != from.pbvrCentreDepth) c.push_back("SetPbvrCentreDepth:" + flag(to.pbvrCentreDepth));
	if (to.pbvrDensityClamp != from.pbvrDensityClamp) c.push_back("SetPbvrDensityClamp:" + flag(to.pbvrDensityClamp));
	if (to.sppSide != from.sppSide) c.push_back("SetGpSpp:" + std::to_string(to.sppSide * to.sppSide));
	if (to.seedMode != from.seedMode) c.push_back(std::string("SetGpSeedMode:") + (to.seedMode == 0 ? "deterministic" : "frame"));
	if (to.densityScale != from.densityScale) c.push_back("SetGpDensityScale:" + fmtArg(to.densityScale));
	if (to.shDegree != from.shDegree) c.push_back("SetGpShDegree:" + std::to_string(to.shDegree));
	if (to.tonemapMode != from.tonemapMode)
		c.push_back(std::string("SetGpTonemap:") + (to.tonemapMode == 1 ? "reinhard" : to.tonemapMode == 2 ? "aces" : "none"));
	if (to.gamma != from.gamma) c.push_back("SetGpGamma:" + fmtArg(to.gamma));
	if (to.pointBudget != from.pointBudget) c.push_back("SetGpPointBudget:" + fmtArg(to.pointBudget));
	if (to.gpPointFootprint != from.gpPointFootprint) c.push_back("SetGpPointFootprint:" + std::to_string(to.gpPointFootprint));
	if (to.gpAdaptiveFootprintKappa != from.gpAdaptiveFootprintKappa || to.gpFootprintMax != from.gpFootprintMax)
		c.push_back("SetGpAdaptiveFootprint:" + fmtArg(to.gpAdaptiveFootprintKappa) + "," + std::to_string(to.gpFootprintMax));
	if (to.gpFootprintCompensate != from.gpFootprintCompensate) c.push_back("SetGpFootprintCompensation:" + flag(to.gpFootprintCompensate));
	return c;
}
} // namespace

GSViewApp::GSViewApp(int width, int height, const std::string& title)
	: ::VKG::VkAppBase(width, height, title)
{
	renderer_.setGSCloud(&gsCloud_);

	// GUI edits take the same queue/handlers as typed commands (see
	// GSViewCommandDispatcher::submitUi), so GUI and Command window cannot diverge.
	panel_.setShell(&shell_);
	panel_.init(
		[this](RenderMode mode) {
			const char* name = mode == RenderMode::GaussianPoint ? "GaussianPoint"
				: mode == RenderMode::PBVR3DExperimental ? "PBVR3DExperimental" : "SortBased";
			dispatcher_.submitUi(std::string("SetRenderMode:") + name);
		},
		[this](float pointSize) {
			dispatcher_.submitUi("SetSortPointSize:" + fmtArg(pointSize));
		},
		[this](float density, int maxP, float pSize, int method) {
			dispatcher_.submitUi("SetDensityScale:" + fmtArg(density));
			dispatcher_.submitUi("SetMaxParticlesPerSplat:" + std::to_string(maxP));
			dispatcher_.submitUi("SetPbvrParticleSize:" + fmtArg(pSize));
			dispatcher_.submitUi("SetPbvr3dMethod:" + std::to_string(method));
		},
		[this](float scale) {
			dispatcher_.submitUi("SetSplatSizeScale:" + fmtArg(scale));
		},
		[this](const GaussianPointRenderer::Params& p) {
			// One command per changed field, same handlers as typed input.
			for (const auto& cmd : gpParamCommands(renderer_.getGaussianPointParams(), p))
				dispatcher_.submitUi(cmd);
		});

	menuBar_.init(
		[this](const std::string& path, std::string& err) {
			return loadPLY(path, err);
		},
		[this]() {
			glfwSetWindowShouldClose(getWindow().get(), GLFW_TRUE);
		});

	add(&renderer_);

	dispatcher_.setApp(this);
	dispatcher_.setRenderer(&renderer_);
	scenarioBrowser_.setHost(this);
	scenarioBrowser_.setDefaultFolder("scenarios");

	// Standard screen: render area + menu + Command + Outliner. Everything
	// else starts hidden and is opened from the View menu / outliner.
	shell_.setDispatcher(&dispatcher_);
	shell_.registerPanel("GSView Control", {0.70f, 0.00f, 0.30f, 0.66f});
	shell_.registerPanel("GS Debug (splat #0)", {0.70f, 0.00f, 0.30f, 0.45f});
	shell_.registerPanel("Scenario Browser", {0.30f, 0.05f, 0.40f, 0.55f});
	shell_.setOutlinerProvider([this] {
		std::vector<ViewShell::OutlinerItem> items;
		if (const size_t n = gsCloud_.points.size())
			items.push_back({0x100000000ull + renderer_.getDataGeneration(),
				"Splat Cloud (" + std::to_string(n) + " splats)", "GS Debug (splat #0)"});
		const RenderMode m = renderer_.getRenderMode();
		items.push_back({1, std::string("Renderer: ") + (m == RenderMode::GaussianPoint ? "Gaussian Point"
			: m == RenderMode::PBVR3DExperimental ? "PBVR 3D (exp.)" : "Sort-Based"), "GSView Control"});
		return items;
	});
	menuBar_.setExtraMenus([this] { shell_.drawViewMenu(); });
}

void GSViewApp::onInit()
{
	static constexpr auto kPR = "shaders/";
	GSViewRenderer::SortShaders s;
	s.pointVert = ::VKG::loadSPVRepo(std::string(kPR) + "point.vert.spv");
	s.pointFrag = ::VKG::loadSPVRepo(std::string(kPR) + "point.frag.spv");
	s.gsVert    = ::VKG::loadSPVRepo(std::string(kPR) + "gs_splat.vert.spv");
	s.gsFrag    = ::VKG::loadSPVRepo(std::string(kPR) + "gs_splat.frag.spv");
	s.gsComp    = ::VKG::loadSPVRepo(std::string(kPR) + "gs_sort.comp.spv");
	s.lineVert  = ::VKG::loadSPVRepo(std::string(kPR) + "line.vert.spv");
	s.lineFrag  = ::VKG::loadSPVRepo(std::string(kPR) + "line.frag.spv");
	renderer_.setSortShaders(std::move(s));

	renderer_.setSortPointSize(panel_.getSortPointSize());
	renderer_.setDensityScale(panel_.getDensityScale());
	renderer_.setMaxParticlesPerSplat(panel_.getMaxParticlesPerSplat());
	renderer_.setPbvrParticleSize(panel_.getPbvrParticleSize());

	::VKG::VkAppBase::onInit();

	renderer_.setExtent(getExtent());
	setupCallbacks();

	if (!initialPLYPath_.empty()) {
		std::string err;
		if (!loadPLY(initialPLYPath_, err))
			std::fprintf(stderr, "[GSView] Failed to load PLY: %s\n", err.c_str());
	}

	if (!initialRenderMode_.empty()) {
		RenderMode mode;
		if (!parseRenderModeName(initialRenderMode_, mode)) {
			std::fprintf(stderr, "[GSView] Unknown --render-mode: %s\n", initialRenderMode_.c_str());
		} else if (mode != RenderMode::SortBased && !renderer_.isGaussianPointAvailable()) {
			std::fprintf(stderr, "[GSView] --render-mode %s unavailable (renderer init failed); staying SortBased\n",
			             initialRenderMode_.c_str());
		} else {
			renderer_.setRenderMode(mode);
		}
	}

	renderer_.setCameraFlipY(initialCameraFlipY_);
	if (hasInitialCamera_) {
		if (!renderer_.setEvaluationCamera(initialCamTheta_, initialCamPhi_, initialCamDistance_))
			std::fprintf(stderr, "[GSView] Invalid --camera-theta/--camera-phi/--camera-distance\n");
	}
}

void GSViewApp::onImGuiReady()
{
	// Context exists, imgui.ini is not read until the first frame.
	shell_.installSettings();
}

void GSViewApp::onUpdate(uint32_t frameIndex)
{
	dispatcher_.processQueue();

	// Single place that collects responses: first the ones for commands typed
	// into the Command window, the rest belong to the running scenario.
	auto responses = dispatcher_.drainResponses();
	shell_.consumeResponses(responses);
	const bool scenarioRunning = runner_.isActive();
	shell_.setScenarioActive(scenarioRunning);
	panel_.setLocked(scenarioRunning);

	if (scenarioRunning) {
		if (runner_.tick(shell_.scenarioDispatcher(), responses)) {
			if (runner_.hasFailed()) {
				std::fprintf(stderr, "[Scenario] FAILED: %s\n", runner_.failMessage().c_str());
				exitCode_ = 1;
			} else {
				std::fprintf(stdout, "[Scenario] PASSED (%zu steps)\n", runner_.stepCount());
				exitCode_ = 0;
			}
			if (exitOnComplete_) getWindow().close();
		}
	} else if (exitAfterScreenshot_ && isScreenshotDone()) {
		getWindow().close();
	}

	::VKG::VkAppBase::onUpdate(frameIndex);
	panel_.setSplatCount(renderer_.getSplatCount());
	panel_.setParticleCount(renderer_.getParticleCount());
	panel_.setParticleCapacity(renderer_.getParticleCapacity());
	panel_.setFPS(ImGui::GetIO().Framerate);
	panel_.setCurrentMode(renderer_.getRenderMode());
	panel_.setDebugSplat(renderer_.getDebugSplat());
	panel_.setGSAvailable(renderer_.isGSAvailable());
	panel_.setSplatSizeScale(renderer_.getSplatSizeScale());
	panel_.setCameraFlipY(renderer_.getCameraFlipY());
	panel_.setGaussianPointAvailable(renderer_.isGaussianPointAvailable());
	panel_.setGaussianPointStats(renderer_.getGaussianPointStats());
	// Mirror renderer state every frame (also while panels are hidden) so
	// command-driven changes are visible when a panel is opened.
	panel_.setGaussianPointParams(renderer_.getGaussianPointParams());
	panel_.syncPbvrParams(dispatcher_.lastSortPointSize(), dispatcher_.lastDensityScale(),
		dispatcher_.lastMaxParticles(), dispatcher_.lastPbvrParticleSize(), renderer_.getPbvr3dMethod());
}

void GSViewApp::onPreRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	renderer_.recordGaussianPointCompute(cmd, frameIndex);
}

void GSViewApp::onSwapChainCreated()
{
	renderer_.setExtent(getExtent());
}

void GSViewApp::onImGui()
{
	if (hideUI_) return; // rendering-only mode: draw nothing so the scene alone is captured
	menuBar_.onImGui();
	shell_.drawWindows();
	panel_.onImGui();
	scenarioBrowser_.pumpQueue();
	if (shell_.beginPanel("Scenario Browser")) {
		scenarioBrowser_.drawEmbedded();
		shell_.endPanel();
	}
	::VKG::VkAppBase::onImGui();
}

void GSViewApp::onCleanup()
{
	::VKG::VkAppBase::onCleanup();
}

bool GSViewApp::loadPLY(const std::string& path, std::string& errorMessage)
{
	Phantom::PointCloud::GSPointCloud loaded;
	if (!loaded.readFromFile(path)) {
		errorMessage = "Failed to read GS-PLY: " + path;
		return false;
	}
	gsCloud_ = std::move(loaded);
	renderer_.setGSCloud(&gsCloud_);
	return true;
}

bool GSViewApp::loadScenario(const std::string& jsonPath)
{
	return runner_.load(jsonPath);
}

bool GSViewApp::publicLoadPLY(const std::string& path, std::string& err, size_t& outCount)
{
	const bool ok = loadPLY(path, err);
	outCount = gsCloud_.points.size();
	return ok;
}

void GSViewApp::setupCallbacks()
{
	auto& win = getWindow();
	// Camera input is ignored while ImGui owns the mouse (typing/selecting in the
	// Command window, dragging sliders) and while a scenario runs, so it cannot
	// disturb a scenario's expectations. A release is always forwarded.
	win.onMouseButton = [this](int button, int action, int) {
		if (button != 0) return;
		if (action == 1 && (ImGui::GetIO().WantCaptureMouse || runner_.isActive())) return;
		renderer_.handleMouseButton(action == 1);
	};
	win.onCursorPos = [this](double x, double y) {
		renderer_.handleMouseMove(x, y);
	};
	win.onScroll = [this](double, double dy) {
		if (ImGui::GetIO().WantCaptureMouse || runner_.isActive()) return;
		renderer_.handleScroll(dy);
	};
	panel_.setOnCameraFlipYChanged([this](bool flip) {
		dispatcher_.submitUi(std::string("SetCameraFlipY:") + (flip ? "1" : "0"));
	});
}

} // namespace GSView
