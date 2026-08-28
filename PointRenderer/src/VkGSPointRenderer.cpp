#include "../include/VkGSPointRenderer.h"

#include "../../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <cstdio>

namespace VKR {

bool VkGSPointRenderer::create(const Phantom::VKG::VulkanContext& ctx,
                               const Phantom::VKG::VulkanCommandPool& pool,
                               VkRenderPass renderPass,
                               const VkGSPointRendererConfig& cfg) {
    ctx_ = &ctx;
    pool_ = &pool;
    framesInFlight_ = cfg.framesInFlight;

    VkDevice dev = ctx.getDevice();

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding = 0;
    uboBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    if (!gfxSetLayout_.create(dev, { uboBinding })) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create graphics descriptor set layout\n");
        return false;
    }

    Phantom::VKG::PipelineConfig pCfg;
    pCfg.vertSpv = cfg.vertSpv;
    pCfg.fragSpv = cfg.fragSpv;
    pCfg.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    pCfg.cullMode = VK_CULL_MODE_NONE;
    pCfg.depthTest = true;
    pCfg.depthWrite = false;
    pCfg.blendEnable = true;
    pCfg.samples = cfg.samples;

    VkVertexInputBindingDescription bd{};
    bd.binding = 0;
    bd.stride = sizeof(GSSplat);
    bd.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    pCfg.bindingDescs = { bd };

    pCfg.attrDescs = {
        VkVertexInputAttributeDescription{ 0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(GSSplat, centerSize)) },
        VkVertexInputAttributeDescription{ 1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(GSSplat, covRow0)) },
        VkVertexInputAttributeDescription{ 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(GSSplat, covRow1)) },
        VkVertexInputAttributeDescription{ 3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(GSSplat, covRow2)) },
        VkVertexInputAttributeDescription{ 4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(GSSplat, color)) }
    };
    pCfg.descriptorSetLayout = gfxSetLayout_.get();
    if (!gfxPipeline_.create(ctx, renderPass, pCfg)) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create graphics pipeline\n");
        return false;
    }

    gfxUbos_.resize(framesInFlight_);
    for (auto& u : gfxUbos_)
        u.createMapped(ctx, sizeof(glm::mat4), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

    VkDescriptorPoolSize uboPool{};
    uboPool.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboPool.descriptorCount = framesInFlight_;
    if (!gfxPool_.create(dev, { uboPool }, framesInFlight_)) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create graphics descriptor pool\n");
        return false;
    }

    std::vector<VkDescriptorSetLayout> layouts(framesInFlight_, gfxSetLayout_.get());
    gfxSets_ = gfxPool_.allocateSets(dev, layouts);
    if (gfxSets_.empty()) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to allocate graphics descriptor sets\n");
        return false;
    }
    for (uint32_t i = 0; i < framesInFlight_; ++i) {
        VkDescriptorBufferInfo bi{};
        bi.buffer = gfxUbos_[i].get();
        bi.offset = 0;
        bi.range = sizeof(glm::mat4);

        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet = gfxSets_[i];
        w.dstBinding = 0;
        w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w.descriptorCount = 1;
        w.pBufferInfo = &bi;
        vkUpdateDescriptorSets(dev, 1, &w, 0, nullptr);
    }

    VkDescriptorSetLayoutBinding storageBinding{};
    storageBinding.binding = 0;
    storageBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    storageBinding.descriptorCount = 1;
    storageBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    if (!computeSetLayout_.create(dev, { storageBinding })) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create compute descriptor set layout\n");
        return false;
    }

    VkDescriptorPoolSize storagePool{};
    storagePool.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    storagePool.descriptorCount = 1;
    if (!computePool_.create(dev, { storagePool }, 1)) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create compute descriptor pool\n");
        return false;
    }

    auto computeSets = computePool_.allocateSets(dev, { computeSetLayout_.get() });
    if (computeSets.empty()) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to allocate compute descriptor set\n");
        return false;
    }
    computeSet_ = computeSets.front();

    Phantom::VKG::ComputePipelineConfig cCfg;
    cCfg.compSpv = cfg.compSpv;
    cCfg.descriptorSetLayout = computeSetLayout_.get();
    cCfg.pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    cCfg.pushConstantRange.offset = 0;
    cCfg.pushConstantRange.size = sizeof(GSSortPushConstants);
    if (!computePipeline_.create(ctx, cCfg)) {
        std::fprintf(stderr, "[VkGSPointRenderer] Failed to create compute pipeline\n");
        return false;
    }
    return true;
}

void VkGSPointRenderer::destroy(VkDevice device) {
    splatBuffer_.destroy(device);

    computePipeline_.destroy(device);
    computePool_.destroy(device);
    computeSetLayout_.destroy(device);
    computeSet_ = VK_NULL_HANDLE;

    for (auto& u : gfxUbos_) u.destroy(device);
    gfxUbos_.clear();
    gfxPool_.destroy(device);
    gfxPipeline_.destroy(device);
    gfxSetLayout_.destroy(device);

    count_ = 0;
    framesInFlight_ = 0;
    ctx_ = nullptr;
    pool_ = nullptr;
}

void VkGSPointRenderer::upload(const Phantom::VKG::VulkanContext& ctx,
                               const std::vector<GSSplat>& splats) {
    VkDevice dev = ctx.getDevice();

    count_ = static_cast<uint32_t>(splats.size());
    if (count_ == 0) {
        splatBuffer_.destroy(dev);
        return;
    }

    const VkDeviceSize bytes = sizeof(GSSplat) * count_;
    if (!splatBuffer_.isValid() || splatBuffer_.getSize() < bytes) {
        splatBuffer_.destroy(dev);
        splatBuffer_.createMapped(ctx,
                                  bytes,
                                  VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        updateComputeDescriptor();
    }

    splatBuffer_.write(splats.data(), bytes);
}

void VkGSPointRenderer::updateMVP(uint32_t frame, const glm::mat4& mvp) {
    if (frame < gfxUbos_.size())
        gfxUbos_[frame].write(&mvp, sizeof(mvp));
}

void VkGSPointRenderer::sortByView(const glm::vec3& eye) {
    if (!ctx_ || !pool_ || count_ < 2 || computeSet_ == VK_NULL_HANDLE) return;

    auto cmd = pool_->beginSingleTimeCommands();

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_.getPipeline());
    vkCmdBindDescriptorSets(cmd,
                            VK_PIPELINE_BIND_POINT_COMPUTE,
                            computePipeline_.getLayout(),
                            0,
                            1,
                            &computeSet_,
                            0,
                            nullptr);

    for (uint32_t phase = 0; phase < count_; ++phase) {
        const uint32_t pairCount = (count_ > phase) ? ((count_ - phase) / 2u) : 0u;
        if (pairCount == 0) break;

        GSSortPushConstants pc{};
        pc.count = count_;
        pc.phase = phase & 1u;
        pc.eye = glm::vec4(eye, 0.0f);

        vkCmdPushConstants(cmd,
                           computePipeline_.getLayout(),
                           VK_SHADER_STAGE_COMPUTE_BIT,
                           0,
                           sizeof(GSSortPushConstants),
                           &pc);

        const uint32_t groups = (pairCount + 255u) / 256u;
        vkCmdDispatch(cmd, groups, 1, 1);

        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0,
                             1,
                             &barrier,
                             0,
                             nullptr,
                             0,
                             nullptr);
    }

    pool_->endSingleTimeCommands(cmd);
}

void VkGSPointRenderer::render(VkCommandBuffer cmd, uint32_t frameIndex) const {
    if (count_ == 0) return;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, gfxPipeline_.getPipeline());

    VkBuffer vbufs[] = { splatBuffer_.get() };
    VkDeviceSize offsets[] = { 0 };
    vkCmdBindVertexBuffers(cmd, 0, 1, vbufs, offsets);

    VkDescriptorSet ds = gfxSets_[frameIndex];
    vkCmdBindDescriptorSets(cmd,
                            VK_PIPELINE_BIND_POINT_GRAPHICS,
                            gfxPipeline_.getLayout(),
                            0,
                            1,
                            &ds,
                            0,
                            nullptr);

    vkCmdDraw(cmd, count_, 1, 0, 0);
}

void VkGSPointRenderer::updateComputeDescriptor() {
    if (!ctx_ || !splatBuffer_.isValid() || computeSet_ == VK_NULL_HANDLE) return;

    VkDescriptorBufferInfo bi{};
    bi.buffer = splatBuffer_.get();
    bi.offset = 0;
    bi.range = VK_WHOLE_SIZE;

    VkWriteDescriptorSet w{};
    w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w.dstSet = computeSet_;
    w.dstBinding = 0;
    w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w.descriptorCount = 1;
    w.pBufferInfo = &bi;
    vkUpdateDescriptorSets(ctx_->getDevice(), 1, &w, 0, nullptr);
}

} // namespace VKR
