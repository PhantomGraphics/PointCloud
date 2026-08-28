#include "VulkanPointCloudPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"

namespace VPC {

void VulkanPointCloudPipeline::create(const Phantom::VKG::VulkanContext& ctx,
                                       VkRenderPass renderPass,
                                       const Config& cfg)
{
    framesInFlight_ = cfg.framesInFlight;
    VkDevice dev = ctx.getDevice();

    // Descriptor set layout: UBO at binding 0 (vertex stage)
    VkDescriptorSetLayoutBinding binding{};
    binding.binding         = 0;
    binding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT;
    descriptorSetLayout_.create(dev, { binding });

    // Graphics pipeline
    Phantom::VKG::PipelineConfig pCfg;
    pCfg.vertSpv             = cfg.vertSpv;
    pCfg.fragSpv             = cfg.fragSpv;
    pCfg.topology            = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    pCfg.cullMode            = VK_CULL_MODE_NONE;
    pCfg.depthTest           = true;
    pCfg.depthWrite          = true;
    pCfg.bindingDescs        = { cfg.bindingDesc };
    pCfg.attrDescs           = cfg.attrDescs;
    pCfg.descriptorSetLayout = descriptorSetLayout_.get();
    pipeline_.create(ctx, renderPass, pCfg);

    // Per-frame host-visible uniform buffers
    uniformBuffers_.resize(framesInFlight_);
    for (auto& ub : uniformBuffers_)
        ub.createMapped(ctx, sizeof(glm::mat4), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

    // Descriptor pool and sets
    VkDescriptorPoolSize ps{};
    ps.type            = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    ps.descriptorCount = framesInFlight_;
    descriptorPool_.create(dev, { ps }, framesInFlight_);

    std::vector<VkDescriptorSetLayout> layouts(framesInFlight_, descriptorSetLayout_.get());
    descriptorSets_ = descriptorPool_.allocateSets(dev, layouts);

    for (uint32_t i = 0; i < framesInFlight_; ++i) {
        VkDescriptorBufferInfo bi{};
        bi.buffer = uniformBuffers_[i].get();
        bi.offset = 0;
        bi.range  = sizeof(glm::mat4);

        VkWriteDescriptorSet w{};
        w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet          = descriptorSets_[i];
        w.dstBinding      = 0;
        w.dstArrayElement = 0;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w.descriptorCount = 1;
        w.pBufferInfo     = &bi;
        vkUpdateDescriptorSets(dev, 1, &w, 0, nullptr);
    }
}

void VulkanPointCloudPipeline::destroy(VkDevice device) {
    for (auto& ub : uniformBuffers_) ub.destroy(device);
    uniformBuffers_.clear();
    descriptorPool_.destroy(device);
    pipeline_.destroy(device);
    descriptorSetLayout_.destroy(device);
    framesInFlight_ = 0;
}

void VulkanPointCloudPipeline::updateUBO(uint32_t frame, const glm::mat4& mvp) {
    uniformBuffers_[frame].write(&mvp, sizeof(mvp));
}

} // namespace VPC
