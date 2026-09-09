#pragma once

// -----------------------------------------------------------------------------
// GaussianPointRenderer -- GPS-style screen-space Gaussian-Point renderer
// (docs/todo/PLAN_gsview_gaussian_point_pbvr.md Phase 2, MVP).
//
// Per frame, entirely on the GPU, no CPU wait / vkDeviceWaitIdle:
//   1. clear the per-subpixel depth/colour/stats buffers (vkCmdFillBuffer)
//   2. gps_splat.comp pass 0  -- project each Gaussian, Poisson-count points,
//      scatter them, atomicMin the nearest depth key per subpixel
//   3. gps_splat.comp pass 1  -- re-scatter, write the winning colour
//   4. gps_resolve.comp       -- average the spp subpixels -> resolved image buf
//   5. gps_composite (graphics, inside the swapchain pass) -- blit to screen
//
// MVP scope: DC colour, single background colour, camera driven by the caller.
// No SH, no temporal accumulation, no occlusion culling, no scan/compaction
// (one thread per Gaussian; padding-free work-lists are Phase 5).
// -----------------------------------------------------------------------------

#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanComputePipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <array>
#include <cstdint>

namespace Phantom::VKG { class VulkanContext; class VulkanCommandPool; }
namespace Phantom::PointCloud { struct GSPointCloud; }

namespace GSView {

class GaussianPointRenderer {
public:
    static constexpr uint32_t kMaxFrames = 2;

    struct Params {
        int   sppSide          = 2;      // subpixel grid side (spp = sppSide^2), 1..4
        int   seedMode         = 1;      // 0 = deterministic, 1 = frame-varying
        int   countMode        = 0;      // 0 = Poisson, 1 = stochastic rounding
        float densityScale     = 1.0f;
        float maxPointsPerSplat = 2048.f;
        glm::vec3 background    = glm::vec3(0.05f);
        float opacityCutoff    = 1.0e-3f;
        float lowPass          = 0.3f;
        float nearZ            = 0.05f;
    };

    struct Camera {
        glm::mat4 view{ 1.0f };   // world -> camera (GLM: camera looks -Z)
        float focalX = 1.0f, focalY = 1.0f;   // pixels
        float cx = 0.0f, cy = 0.0f;           // principal point (pixels)
    };

    struct Stats {
        uint32_t expectedCount  = 0;   // sum of per-Gaussian E[N], rounded
        uint32_t generatedCount = 0;   // sum of Poisson counts
        uint32_t activeSamples  = 0;   // covered subpixels after resolve
        uint32_t drawnPoints    = 0;   // points that landed on screen
    };

    void onInit(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight);
    void onCleanup(VkDevice device);

    // Recreate extent-dependent buffers. Call from the app's onSwapChainCreated
    // and once after onInit.
    void onResize(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                  VkExtent2D extent);

    void setGSCloud(const Phantom::PointCloud::GSPointCloud* cloud) { cloud_ = cloud; }
    void setParams(const Params& p);
    void setCamera(const Camera& c) { camera_ = c; }

    bool isAvailable() const { return available_; }
    const char* backendName() const { return "32-bit two-pass"; }
    Stats getStats() const;

    // Called from GSViewRenderer::onUpdate: uploads the input SSBO if the cloud
    // changed and writes the per-frame params UBO. No command recording.
    void update(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                uint32_t frameIndex);

    // Called from the app's onPreRender (before the swapchain render pass).
    void recordCompute(VkCommandBuffer cmd, uint32_t frameIndex);

    // Called from GSViewRenderer::onRender (inside the swapchain render pass).
    void recordComposite(VkCommandBuffer cmd, uint32_t frameIndex);

private:
    struct ParamsUBO {
        glm::mat4  view;
        glm::vec4  p0;    // focalX, focalY, cx, cy
        glm::vec4  p1;    // nearZ, lowPass, shC0, opacityCutoff
        glm::uvec4 dims;  // width, height, spp, sppSide
        glm::uvec4 ctrl;  // numSplats, frameIndex, seedMode, countMode
        glm::vec4  p2;    // maxPointsPerSplat, pointBudgetScale, densityScale, 0
        glm::vec4  bg;    // background rgb, 0
    };
    struct PushConstants { uint32_t pass; };

    bool available_ = false;
    Params params_;
    Camera camera_;
    const Phantom::PointCloud::GSPointCloud* cloud_ = nullptr;

    const Phantom::VKG::VulkanContext* ctx_ = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    uint32_t frames_ = kMaxFrames;
    uint32_t frameCounter_ = 0;
    uint32_t lastFrameIndex_ = 0;

    VkExtent2D extent_{ 0, 0 };
    uint32_t   spp_ = 4;

    // input
    Phantom::VKG::VulkanBuffer gsInput_;
    uint32_t numSplats_ = 0;
    uint64_t cachedGeneration_ = ~0ull;

    // per-frame buffers
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> depthBuf_;
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> colorBuf_;
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> resolvedBuf_;
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> statsBuf_;    // host-visible
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> paramsUbo_;   // host-visible

    // compute
    Phantom::VKG::VulkanDescriptorSetLayout computeDsl_;
    Phantom::VKG::VulkanComputePipeline     splatPipe_;
    Phantom::VKG::VulkanComputePipeline     resolvePipe_;

    // composite (graphics)
    Phantom::VKG::VulkanDescriptorSetLayout compositeDsl_;
    Phantom::VKG::VulkanPipeline            compositePipe_;

    Phantom::VKG::VulkanDescriptorPool           descPool_;
    std::array<VkDescriptorSet, kMaxFrames>      computeSets_{};
    std::array<VkDescriptorSet, kMaxFrames>      compositeSets_{};

    void destroyFrameBuffers(VkDevice device);
    void writeComputeSet(VkDevice device, uint32_t f);
    void writeCompositeSet(VkDevice device, uint32_t f);
};

} // namespace GSView
