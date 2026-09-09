#include "GSViewApp.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

#include <GLFW/glfw3.h>
#include <cstdio>

namespace GSView {

GSViewApp::GSViewApp(int width, int height, const std::string& title)
	: ::VKG::VkAppBase(width, height, title)
{
	renderer_.setGSCloud(&gsCloud_);

	panel_.init(
		[this](RenderMode mode) {
			renderer_.setRenderMode(mode);
		},
		[this](float pointSize) {
			renderer_.setSortPointSize(pointSize);
		},
		[this](float density, int maxP, float pSize) {
			renderer_.setDensityScale(density);
			renderer_.setMaxParticlesPerSplat(maxP);
			renderer_.setPbvrParticleSize(pSize);
		},
		[this](float scale) {
			renderer_.setSplatSizeScale(scale);
		},
		[this](const GaussianPointRenderer::Params& p) {
			renderer_.setGaussianPointParams(p);
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
	add(&scenarioBrowser_);
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
	panel_.setGaussianPointParams(renderer_.getGaussianPointParams());
	setupCallbacks();

	if (!initialPLYPath_.empty()) {
		std::string err;
		if (!loadPLY(initialPLYPath_, err))
			std::fprintf(stderr, "[GSView] Failed to load PLY: %s\n", err.c_str());
	}
}

void GSViewApp::onUpdate(uint32_t frameIndex)
{
	dispatcher_.processQueue();

	if (runner_.isActive()) {
		auto responses = dispatcher_.drainResponses();
		if (runner_.tick(dispatcher_, responses)) {
			if (runner_.hasFailed()) {
				std::fprintf(stderr, "[Scenario] FAILED: %s\n", runner_.failMessage().c_str());
				exitCode_ = 1;
			} else {
				std::fprintf(stdout, "[Scenario] PASSED (%zu steps)\n", runner_.stepCount());
				exitCode_ = 0;
			}
			if (exitOnComplete_) getWindow().close();
		}
	} else {
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
	panel_.setGaussianPointAvailable(renderer_.isGaussianPointAvailable());
	panel_.setGaussianPointStats(renderer_.getGaussianPointStats());
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
	menuBar_.onImGui();
	panel_.onImGui();
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
	win.onMouseButton = [this](int button, int action, int) {
		if (button == 0) renderer_.handleMouseButton(action == 1);
	};
	win.onCursorPos = [this](double x, double y) {
		renderer_.handleMouseMove(x, y);
	};
	win.onScroll = [this](double, double dy) {
		renderer_.handleScroll(dy);
	};
}

} // namespace GSView
