#pragma once

#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanComputePipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

#include <vulkan/vulkan.h>
#include <cstdint>

namespace Phantom::VKG {
class VulkanContext;
class VulkanCommandPool;
}

namespace Phantom::PointCloud {
struct GSPointCloud;
}

namespace GSView {

class GSComputePBVR {
public:
    void create(const Phantom::VKG::VulkanContext& ctx);
    void destroy(VkDevice device);

    void setParams(float densityScale, int maxParticlesPerSplat);

    // GPU particle generation. Called from regeneratePBVR() when dirty.
    // Blocks until GPU is idle (beginSingleTimeCommands / endSingleTimeCommands).
    void dispatch(const Phantom::VKG::VulkanContext& ctx,
                  const Phantom::VKG::VulkanCommandPool& pool,
                  const Phantom::PointCloud::GSPointCloud& cloud);

    VkBuffer getVertexBuffer()     const { return outputBuf_.getBuffer(); }
    uint32_t getTotalVertexCount() const { return totalCount_; }
    bool     isValid()             const { return pipeline_.isValid(); }

private:
    struct PushConstants {
        uint32_t numSplats;
        uint32_t maxParticlesPerSplat;
        float    densityScale;
        uint32_t _pad;
    };

    Phantom::VKG::VulkanBuffer              inputBuf_;   // SSBO: GSPoint[]
    Phantom::VKG::VulkanBuffer              outputBuf_;  // SSBO + VBO: GpuVertex[]
    Phantom::VKG::VulkanDescriptorSetLayout dsl_;
    Phantom::VKG::VulkanDescriptorPool      descPool_;
    VkDescriptorSet                descSet_  = VK_NULL_HANDLE;
    Phantom::VKG::VulkanComputePipeline     pipeline_;

    uint32_t totalCount_      = 0;
    uint32_t cachedNumSplats_ = 0;
    uint32_t maxPPS_          = 8;
    float    densityScale_    = 1.0f;

    void rebuildInputBuf(const Phantom::VKG::VulkanContext& ctx,
                         const Phantom::VKG::VulkanCommandPool& pool,
                         const Phantom::PointCloud::GSPointCloud& cloud);
    void rebuildOutputBuf(const Phantom::VKG::VulkanContext& ctx,
                          const Phantom::VKG::VulkanCommandPool& pool,
                          uint32_t numSplats);
    void updateDescSet(VkDevice device);
};

} // namespace GSView
