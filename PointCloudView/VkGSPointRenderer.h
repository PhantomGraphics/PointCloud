#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "GSSplat.h"

#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanComputePipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

#include <vector>

namespace Phantom::VKG {
class VulkanContext;
class VulkanCommandPool;
}

namespace VPC {

class VkGSPointRenderer {
public:
    struct Config {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
        std::vector<uint32_t> compSpv;
        uint32_t framesInFlight = 2;
        VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
    };

    void create(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass,
                const Config& cfg);
    void destroy(VkDevice device);

    void upload(const Phantom::VKG::VulkanContext& ctx,
                const std::vector<GSSplat>& splats);

    void updateMVP(uint32_t frame, const glm::mat4& mvp);
    void sortByView(const glm::vec3& eye);

    void render(VkCommandBuffer cmd, uint32_t frameIndex) const;

    uint32_t getCount() const { return count_; }
    bool hasData() const { return count_ > 0; }

private:
    const Phantom::VKG::VulkanContext* ctx_ = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    uint32_t framesInFlight_ = 0;
    uint32_t count_ = 0;

    Phantom::VKG::VulkanDescriptorSetLayout gfxSetLayout_;
    Phantom::VKG::VulkanDescriptorPool gfxPool_;
    std::vector<VkDescriptorSet> gfxSets_;
    std::vector<Phantom::VKG::VulkanBuffer> gfxUbos_;
    Phantom::VKG::VulkanPipeline gfxPipeline_;

    Phantom::VKG::VulkanDescriptorSetLayout computeSetLayout_;
    Phantom::VKG::VulkanDescriptorPool computePool_;
    VkDescriptorSet computeSet_ = VK_NULL_HANDLE;
    Phantom::VKG::VulkanComputePipeline computePipeline_;

    Phantom::VKG::VulkanBuffer splatBuffer_;

    void updateComputeDescriptor();
};

} // namespace VPC
