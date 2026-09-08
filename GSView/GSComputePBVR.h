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

    VkBuffer getVertexBuffer() const { return outputBuf_.getBuffer(); }

    // --- Three distinct counts (Phase 0, step 2) --------------------------------
    // capacity      : vertices the output buffer can hold  = numSplats * maxParticlesPerSplat.
    // generatedCount: sum of the per-splat particle counts actually produced this dispatch
    //                 (can be 0 when every splat is fully transparent).
    // drawCount     : vertices handed to vkCmdDraw. The current single-pass shader writes one
    //                 slot per capacity entry (padding slots get alpha=0 and are culled in the
    //                 vertex shader), so this equals capacity while any particle exists, else 0.
    //                 Compaction to exactly generatedCount is Phase 2 work.
    uint32_t getCapacity()       const { return capacity_; }
    uint32_t getGeneratedCount() const { return generatedCount_; }
    uint32_t getDrawCount()      const { return drawCount_; }

    bool     isValid()           const { return pipeline_.isValid(); }

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

    uint32_t capacity_         = 0;
    uint32_t generatedCount_   = 0;
    uint32_t drawCount_        = 0;
    uint32_t cachedNumSplats_  = 0;
    uint64_t cachedGeneration_ = 0;
    uint32_t maxPPS_           = 8;
    float    densityScale_     = 1.0f;

    void rebuildInputBuf(const Phantom::VKG::VulkanContext& ctx,
                         const Phantom::VKG::VulkanCommandPool& pool,
                         const Phantom::PointCloud::GSPointCloud& cloud);
    void rebuildOutputBuf(const Phantom::VKG::VulkanContext& ctx,
                          const Phantom::VKG::VulkanCommandPool& pool,
                          uint32_t numSplats);
    void updateDescSet(VkDevice device);
};

} // namespace GSView
