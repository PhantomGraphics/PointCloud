#include "GSViewRenderer.h"

#include "../PointCloud/GSPointCloud.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

static glm::vec3 rowFromQuat0(const float* q) {
	const float x = q[1], y = q[2], z = q[3], w = q[0];
	return { 1.0f - 2.0f*(y*y+z*z), 2.0f*(x*y-z*w), 2.0f*(x*z+y*w) };
}
static glm::vec3 rowFromQuat1(const float* q) {
	const float x = q[1], y = q[2], z = q[3], w = q[0];
	return { 2.0f*(x*y+z*w), 1.0f - 2.0f*(x*x+z*z), 2.0f*(y*z-x*w) };
}
static glm::vec3 rowFromQuat2(const float* q) {
	const float x = q[1], y = q[2], z = q[3], w = q[0];
	return { 2.0f*(x*z-y*w), 2.0f*(y*z+x*w), 1.0f - 2.0f*(x*x+y*y) };
}
static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
static float shDcToColor(float sh) {
	return std::clamp(sh * 0.28209479177387814f + 0.5f, 0.0f, 1.0f);
}

static std::vector<VKR::GSSplat> buildSplats(
	const Phantom::PointCloud::GSPointCloud& cloud,
	GSView::DebugSplatInfo& dbg,
	float splatSizeScale)
{
	dbg = {};
	std::vector<VKR::GSSplat> out;
	out.reserve(cloud.points.size());

	constexpr int kPrintCount = 3;
	int printIdx = 0;

	for (const auto& p : cloud.points) {
		const float sx       = std::exp(p.scale[0]);
		const float sy       = std::exp(p.scale[1]);
		const float sz       = std::exp(p.scale[2]);
		const float maxScale = std::max({ sx, sy, sz });

		const glm::vec3 r0 = rowFromQuat0(p.rot);
		const glm::vec3 r1 = rowFromQuat1(p.rot);
		const glm::vec3 r2 = rowFromQuat2(p.rot);

		// vMatrix = maxScale * S^{-1} * R^T stored as columns for GLSL mat3.
		// This normalises the largest axis to fit the point sprite (magnitude 1),
		// so smaller axes are proportionally narrower — rotation is preserved.
		const float msx = maxScale / sx;
		const float msy = maxScale / sy;
		const float msz = maxScale / sz;

		VKR::GSSplat s{};
		s.centerSize = { p.x, p.y, p.z, maxScale * splatSizeScale };
		s.covRow0 = { r0.x * msx, r0.y * msy, r0.z * msz, 0.0f };
		s.covRow1 = { r1.x * msx, r1.y * msy, r1.z * msz, 0.0f };
		s.covRow2 = { r2.x * msx, r2.y * msy, r2.z * msz, 0.0f };
		s.color = {
			shDcToColor(p.f_dc[0]),
			shDcToColor(p.f_dc[1]),
			shDcToColor(p.f_dc[2]),
			sigmoid(p.opacity)
		};

		if (out.empty()) {
			dbg.sx = sx; dbg.sy = sy; dbg.sz = sz; dbg.maxScale = maxScale;
			for (int i = 0; i < 3; ++i) dbg.rawScale[i] = p.scale[i];
			for (int i = 0; i < 4; ++i) dbg.rawQuat[i]  = p.rot[i];
			dbg.covRow0   = glm::vec3(s.covRow0);
			dbg.covRow1   = glm::vec3(s.covRow1);
			dbg.covRow2   = glm::vec3(s.covRow2);
			dbg.pointSize = maxScale * splatSizeScale;
			dbg.valid     = true;
		}

		if (printIdx < kPrintCount) {
			std::printf("[GS Debug] splat #%d  pos=(%.3f,%.3f,%.3f)\n",
				printIdx, p.x, p.y, p.z);
			std::printf("           log_scale=(%.3f,%.3f,%.3f)  exp=(%.5f,%.5f,%.5f)  max=%.5f\n",
				p.scale[0], p.scale[1], p.scale[2], sx, sy, sz, maxScale);
			std::printf("           quat(w,x,y,z)=(%.4f,%.4f,%.4f,%.4f)\n",
				p.rot[0], p.rot[1], p.rot[2], p.rot[3]);
			std::printf("           msx/msy/msz=(%.4f,%.4f,%.4f)\n", msx, msy, msz);
			std::printf("           covRow0=(%.4f,%.4f,%.4f)\n",
				s.covRow0.x, s.covRow0.y, s.covRow0.z);
			std::printf("           covRow1=(%.4f,%.4f,%.4f)\n",
				s.covRow1.x, s.covRow1.y, s.covRow1.z);
			std::printf("           covRow2=(%.4f,%.4f,%.4f)\n",
				s.covRow2.x, s.covRow2.y, s.covRow2.z);
			std::printf("           opacity(raw)=%.4f  color=(%.2f,%.2f,%.2f)\n",
				p.opacity, s.color.x, s.color.y, s.color.z);
			std::fflush(stdout);
			++printIdx;
		}

		out.push_back(s);
	}
	return out;
}

} // namespace

namespace GSView {

void GSViewRenderer::setGSCloud(const Phantom::PointCloud::GSPointCloud* cloud)
{
	gsCloud_ = cloud;
	sceneDirty_ = true;
	gaussianPoint_.setGSCloud(cloud);
}

static bool isGpMode(RenderMode m)
{
	return m == RenderMode::GaussianPoint || m == RenderMode::PBVR3DExperimental;
}

void GSViewRenderer::setRenderMode(RenderMode mode)
{
	if (isGpMode(mode) && mode_ != mode)
		gaussianPoint_.resetAccumulation();
	mode_ = mode;
}

void GSViewRenderer::setGaussianPointParams(const GaussianPointRenderer::Params& p)
{
	const bool sppChanged = p.sppSide != gpParams_.sppSide;
	gpParams_ = p;
	gaussianPoint_.setParams(gpParams_);
	if (sppChanged) gpExtentDirty_ = true;  // subpixel buffers are sized by spp
}

void GSViewRenderer::recordGaussianPointCompute(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (isGpMode(mode_))
		gaussianPoint_.recordCompute(cmd, frameIndex);
}

void GSViewRenderer::setSortPointSize(float s)
{
	sortRenderer_.setPointSize(std::max(1.0f, s));
}

// PBVR3DExperimental knobs mapped onto the shared GaussianPoint params.
void GSViewRenderer::setDensityScale(float s)
{
	gpParams_.densityScale = std::max(0.1f, s);
	gaussianPoint_.setParams(gpParams_);
}

void GSViewRenderer::setMaxParticlesPerSplat(int n)
{
	gpParams_.maxPointsPerSplat = static_cast<float>(std::max(1, n));
	gaussianPoint_.setParams(gpParams_);
}

void GSViewRenderer::setPbvrParticleSize(float s)
{
	// Repurposed: base particles per splat (the "point size" knob is gone -- the
	// on-screen density is what matters, see the Phase 4 plan).
	gpParams_.basePointsPerSplat = std::max(1.0f, s * 64.0f);
	gaussianPoint_.setParams(gpParams_);
}

void GSViewRenderer::setPbvr3dMethod(int method)
{
	gpParams_.pbvr3dMethod = std::clamp(method, 0, 2);
	gaussianPoint_.setParams(gpParams_);
}

uint32_t GSViewRenderer::getSplatCount() const
{
	return gsCloud_ ? static_cast<uint32_t>(gsCloud_->points.size()) : 0u;
}

uint64_t GSViewRenderer::getDataGeneration() const
{
	return gsCloud_ ? gsCloud_->generation : 0u;
}

void GSViewRenderer::handleMouseButton(bool leftPressed)
{
	isDragging_ = leftPressed;
}

void GSViewRenderer::handleMouseMove(double x, double y)
{
	if (isDragging_) {
		const float dx = static_cast<float>(x - lastX_) * 0.005f;
		const float dy = static_cast<float>(y - lastY_) * 0.005f;
		camPhi_ -= dx;
		camTheta_ = std::max(0.05f, std::min(3.09f, camTheta_ + dy));
	}
	lastX_ = x;
	lastY_ = y;
}

void GSViewRenderer::handleScroll(double dy)
{
	camDist_ = std::max(0.1f, camDist_ - static_cast<float>(dy) * 0.2f);
}

void GSViewRenderer::onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
							VkRenderPass renderPass, uint32_t framesInFlight)
{
	ctx_ = &ctx;
	pool_ = &pool;

	sortRenderer_.setScene(&vkScene_);
	sortRenderer_.setRenderMode(VKR::PointRenderMode::GaussianSplatting);
	syncSortScene();
	sortRenderer_.onInit(ctx, pool, renderPass, framesInFlight, std::move(sortShaders_));
	sceneDirty_ = false;

	gaussianPoint_.onInit(ctx, pool, renderPass, framesInFlight);
	gaussianPoint_.setParams(gpParams_);
	gaussianPoint_.setGSCloud(gsCloud_);
	gpExtentDirty_ = true;
}

void GSViewRenderer::onUpdate(uint32_t frameIndex)
{
	if (!ctx_ || !pool_) return;

	if (sceneDirty_) {
		syncSortScene();
		sceneDirty_ = false;
	}

	const glm::mat4 mvp = computeMVP();
	const glm::vec3 eye = computeEye();
	sortRenderer_.onUpdate(frameIndex, mvp, eye);

	// GaussianPoint / PBVR3D pipeline: (re)create extent-dependent buffers, then push camera + params.
	gaussianPoint_.setPath(mode_ == RenderMode::PBVR3DExperimental
	                           ? GaussianPointRenderer::Path::Pbvr3d
	                           : GaussianPointRenderer::Path::GaussianPoint);
	if (gpExtentDirty_ && extent_.width > 0 && extent_.height > 0) {
		gaussianPoint_.onResize(*ctx_, *pool_, extent_);
		gpExtentDirty_ = false;
	}
	{
		GaussianPointRenderer::Camera cam;
		cam.view = glm::lookAt(eye, camTarget_, glm::vec3(0.f, 1.f, 0.f));
		cam.camPos = eye;
		const float tanFovY = std::tan(glm::radians(45.f) * 0.5f);
		cam.focalY = (static_cast<float>(extent_.height) * 0.5f) / tanFovY;
		cam.focalX = cam.focalY;
		cam.cx = static_cast<float>(extent_.width) * 0.5f;
		cam.cy = static_cast<float>(extent_.height) * 0.5f;
		gaussianPoint_.setCamera(cam);
		gaussianPoint_.update(*ctx_, *pool_, frameIndex);
	}
}

void GSViewRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
	if (mode_ == RenderMode::SortBased) {
		sortRenderer_.onRender(cmd, frameIndex);
		return;
	}
	// GaussianPoint and PBVR3DExperimental share the compute pipeline + composite.
	gaussianPoint_.recordComposite(cmd, frameIndex);
}

void GSViewRenderer::onCleanup(VkDevice device)
{
	sortRenderer_.onCleanup(device);
	gaussianPoint_.onCleanup(device);
	ctx_ = nullptr;
	pool_ = nullptr;
}

void GSViewRenderer::syncSortScene()
{
	vkScene_.clear();
	if (!gsCloud_ || gsCloud_->points.empty()) {
		sortRenderer_.notifySceneChanged();
		return;
	}
	const int id = vkScene_.add("gs_cloud");
	vkScene_.setGSSplats(id, buildSplats(*gsCloud_, debugSplat_, splatSizeScale_));
	vkScene_.setVisible(id, true);
	sortRenderer_.setActiveScene(id);
}

glm::mat4 GSViewRenderer::computeMVP() const
{
	const glm::vec3 eye = computeEye();
	const glm::mat4 view = glm::lookAt(eye, camTarget_, glm::vec3(0.f, 1.f, 0.f));
	const float aspect = (extent_.height > 0)
		? static_cast<float>(extent_.width) / static_cast<float>(extent_.height)
		: 1.f;
	glm::mat4 proj = glm::perspective(glm::radians(45.f), aspect, 0.01f, 100.f);
	proj[1][1] *= -1.f;
	return proj * view;
}

glm::vec3 GSViewRenderer::computeEye() const
{
	const float x = camDist_ * sinf(camTheta_) * cosf(camPhi_);
	const float y = camDist_ * cosf(camTheta_);
	const float z = camDist_ * sinf(camTheta_) * sinf(camPhi_);
	return camTarget_ + glm::vec3(x, y, z);
}

} // namespace GSView
