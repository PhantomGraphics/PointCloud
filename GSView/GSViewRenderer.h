#pragma once

#include "GSComputePBVR.h"

#include "../PointRenderer/include/VkPointRenderer.h"
#include "../PointRenderer/include/VkPointScene.h"
#include "../../CGLib/Volume/VolumeRenderer/PBVRPipeline.h"
#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Phantom::PointCloud { struct GSPointCloud; }

namespace GSView {

// Render paths. PBVR3DExperimental is the object-space "3D Gaussian -> world-space
// particles" prototype (renamed from the old "PBVR" mode in Phase 0 of
// docs/todo/PLAN_gsview_gaussian_point_pbvr.md). Screen-space GPS / ReferenceSplat
// modes are added in later phases.
enum class RenderMode { SortBased, PBVR3DExperimental };

struct DebugSplatInfo {
    float rawScale[3]  = {};
    float rawQuat[4]   = {};
    float sx = 0.f, sy = 0.f, sz = 0.f, maxScale = 0.f;
    glm::vec3 covRow0{}, covRow1{}, covRow2{};
    float pointSize = 0.f;
    bool valid = false;
};

class GSViewRenderer : public ::VKG::IVkSubRenderer {
public:
	using SortShaders = VKR::VkPointRenderer::Shaders;

	void setSortShaders(SortShaders s) { sortShaders_ = std::move(s); }
	void setGSCloud(const Phantom::PointCloud::GSPointCloud* cloud);
	void setRenderMode(RenderMode mode) { mode_ = mode; }
	RenderMode getRenderMode() const { return mode_; }

	void setSortPointSize(float s);
	void setSplatSizeScale(float s) { splatSizeScale_ = std::max(10.f, s); sceneDirty_ = true; }
	float getSplatSizeScale() const { return splatSizeScale_; }
	void setDensityScale(float s);
	void setMaxParticlesPerSplat(int n);
	void setPbvrParticleSize(float s);
	size_t getParticleCount() const { return computePBVR_.getGeneratedCount(); }
	size_t getParticleCapacity() const { return computePBVR_.getCapacity(); }
	uint32_t getSplatCount() const;
	uint64_t getDataGeneration() const;
	const DebugSplatInfo& getDebugSplat() const { return debugSplat_; }
	bool isGSAvailable() const { return sortRenderer_.isGaussianSplattingAvailable(); }

	void handleMouseButton(bool leftPressed);
	void handleMouseMove(double x, double y);
	void handleScroll(double dy);
	void setExtent(VkExtent2D ext) { extent_ = ext; }

	void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
				VkRenderPass renderPass, uint32_t framesInFlight) override;
	void onUpdate(uint32_t frameIndex) override;
	void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
	void onCleanup(VkDevice device) override;

private:
	SortShaders sortShaders_;
	VKR::VkPointScene vkScene_;
	VKR::VkPointRenderer sortRenderer_;
	bool sceneDirty_ = true;

	GSComputePBVR computePBVR_;
	Phantom::Volume::PBVRPipeline pbvrPipeline_;
	float densityScale_         = 1.0f;
	int   maxParticlesPerSplat_ = 8;
	float pbvrParticleSize_     = 4.0f;
	bool  pbvrDirty_            = true;

	float camTheta_ = 0.4f;
	float camPhi_ = 0.5f;
	float camDist_ = 3.0f;
	glm::vec3 camTarget_{ 0.f, 0.f, 0.f };
	bool isDragging_ = false;
	double lastX_ = 0.0;
	double lastY_ = 0.0;

	float splatSizeScale_ = 1000.f;
	RenderMode mode_ = RenderMode::SortBased;
	const Phantom::PointCloud::GSPointCloud* gsCloud_ = nullptr;
	DebugSplatInfo debugSplat_;
	VkExtent2D extent_{ 1280, 720 };
	const Phantom::VKG::VulkanContext* ctx_ = nullptr;
	const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

	void syncSortScene();
	void regeneratePBVR();
	glm::mat4 computeMVP() const;
	glm::vec3 computeEye() const;
};

} // namespace GSView
