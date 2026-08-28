#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <vulkan/vulkan.h>
#include "../../CGLib/VulkanGraphics/VulkanPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"

#include <cstdint>
#include <vector>

namespace Phantom::VKG { class VulkanContext; }

namespace VPC {

// Encapsulates the full Vulkan pipeline for point cloud rendering:
// descriptor set layout, UBOs, descriptor pool/sets, and graphics pipeline.
class VulkanPointCloudPipeline {
public:
    struct Config {
        std::vector<uint32_t> vertSpv;
        std::vector<uint32_t> fragSpv;
        VkVertexInputBindingDescription              bindingDesc{};
        std::vector<VkVertexInputAttributeDescription> attrDescs;
        uint32_t framesInFlight = 2;
    };

    void create(const Phantom::VKG::VulkanContext& ctx, VkRenderPass renderPass, const Config& cfg);
    void destroy(VkDevice device);

    void updateUBO(uint32_t frame, const glm::mat4& mvp);

    VkPipeline       getPipeline()                const { return pipeline_.getPipeline(); }
    VkPipelineLayout getLayout()                  const { return pipeline_.getLayout(); }
    VkDescriptorSet  getDescriptorSet(uint32_t f) const { return descriptorSets_[f]; }
    bool             isValid()                    const { return pipeline_.getPipeline() != VK_NULL_HANDLE; }

private:
    uint32_t framesInFlight_ = 0;
    Phantom::VKG::VulkanDescriptorSetLayout descriptorSetLayout_;
    Phantom::VKG::VulkanDescriptorPool      descriptorPool_;
    std::vector<VkDescriptorSet>   descriptorSets_;
    Phantom::VKG::VulkanPipeline            pipeline_;
    std::vector<Phantom::VKG::VulkanBuffer> uniformBuffers_;
};

} // namespace VPC
