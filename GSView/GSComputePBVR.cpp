#include "GSComputePBVR.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../PointCloud/GSPointCloud.h"
#include "../../CGLib/Volume/VolumeRenderer/PBVRPipeline.h"

#include <stdexcept>
#include <cstddef>

// Layout must match the GLSL shader structs exactly.
static_assert(sizeof(Phantom::PointCloud::GSPoint) == 68,
    "GSPoint layout changed — update gs_pbvr_gen.comp accordingly");
static_assert(sizeof(Phantom::Volume::PBVRVertex) == 28,
    "PBVRVertex layout changed — update gs_pbvr_gen.comp accordingly");
static_assert(offsetof(Phantom::Volume::PBVRVertex, color) == 12,
    "PBVRVertex color offset changed — update gs_pbvr_gen.comp accordingly");

namespace GSView {

void GSComputePBVR::create(const Phantom::VKG::VulkanContext& ctx)
{
    VkDevice dev = ctx.getDevice();

    // Descriptor set layout: binding 0 = input SSBO, binding 1 = output SSBO
    VkDescriptorSetLayoutBinding b0{};
    b0.binding         = 0;
    b0.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    b0.descriptorCount = 1;
    b0.stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutBinding b1 = b0;
    b1.binding = 1;

    dsl_.create(dev, {b0, b1});

    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 2};
    descPool_.create(dev, {ps}, 1);
    auto sets = descPool_.allocateSets(dev, {dsl_.get()});
    descSet_ = sets.front();

    Phantom::VKG::ComputePipelineConfig cfg{};
    cfg.compSpv             = ::VKG::loadSPVRepo("shaders/gs_pbvr_gen.comp.spv");
    cfg.descriptorSetLayout = dsl_.get();
    cfg.pushConstantRange   = {VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants)};
    pipeline_.create(ctx, cfg);
}

void GSComputePBVR::destroy(VkDevice device)
{
    pipeline_.destroy(device);
    descPool_.destroy(device);
    dsl_.destroy(device);
    descSet_ = VK_NULL_HANDLE;
    outputBuf_.destroy(device);
    inputBuf_.destroy(device);
    totalCount_      = 0;
    cachedNumSplats_ = 0;
}

void GSComputePBVR::setParams(float densityScale, int maxParticlesPerSplat)
{
    densityScale_ = densityScale;
    maxPPS_       = static_cast<uint32_t>(maxParticlesPerSplat);
}

void GSComputePBVR::dispatch(const Phantom::VKG::VulkanContext& ctx,
                              const Phantom::VKG::VulkanCommandPool& pool,
                              const Phantom::PointCloud::GSPointCloud& cloud)
{
    if (cloud.points.empty()) {
        totalCount_ = 0;
        return;
    }

    const uint32_t numSplats = static_cast<uint32_t>(cloud.points.size());

    // Rebuild input SSBO when cloud size changes
    if (numSplats != cachedNumSplats_) {
        rebuildInputBuf(ctx, pool, cloud);
        cachedNumSplats_ = numSplats;
    }

    // Rebuild output buffer when capacity is insufficient
    const uint32_t neededVerts = numSplats * maxPPS_;
    const VkDeviceSize neededBytes =
        static_cast<VkDeviceSize>(neededVerts) * sizeof(Phantom::Volume::PBVRVertex);
    if (!outputBuf_.isValid() || outputBuf_.getSize() < neededBytes) {
        rebuildOutputBuf(ctx, pool, numSplats);
    }

    totalCount_ = neededVerts;

    PushConstants pc{numSplats, maxPPS_, densityScale_, 0u};

    VkCommandBuffer cmd = pool.beginSingleTimeCommands();

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_.getPipeline());
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                            pipeline_.getLayout(), 0, 1, &descSet_, 0, nullptr);
    vkCmdPushConstants(cmd, pipeline_.getLayout(),
                       VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);

    const uint32_t groups = (numSplats + 63u) / 64u;
    vkCmdDispatch(cmd, groups, 1, 1);

    // Barrier: compute SSBO write -> vertex attribute read
    VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    vkCmdPipelineBarrier(cmd,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        0, 1, &barrier, 0, nullptr, 0, nullptr);

    pool.endSingleTimeCommands(cmd);
}

void GSComputePBVR::rebuildInputBuf(const Phantom::VKG::VulkanContext& ctx,
                                     const Phantom::VKG::VulkanCommandPool& pool,
                                     const Phantom::PointCloud::GSPointCloud& cloud)
{
    inputBuf_.destroy(ctx.getDevice());
    const VkDeviceSize bytes =
        static_cast<VkDeviceSize>(cloud.points.size()) * sizeof(Phantom::PointCloud::GSPoint);
    inputBuf_.create(ctx, pool, bytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     cloud.points.data());
    updateDescSet(ctx.getDevice());
}

void GSComputePBVR::rebuildOutputBuf(const Phantom::VKG::VulkanContext& ctx,
                                      const Phantom::VKG::VulkanCommandPool& pool,
                                      uint32_t numSplats)
{
    outputBuf_.destroy(ctx.getDevice());
    const VkDeviceSize bytes =
        static_cast<VkDeviceSize>(numSplats) * maxPPS_ * sizeof(Phantom::Volume::PBVRVertex);
    outputBuf_.create(ctx, pool, bytes,
                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                      nullptr);
    updateDescSet(ctx.getDevice());
}

void GSComputePBVR::updateDescSet(VkDevice device)
{
    if (!inputBuf_.isValid() || !outputBuf_.isValid()) return;

    VkDescriptorBufferInfo inInfo{inputBuf_.getBuffer(),  0, VK_WHOLE_SIZE};
    VkDescriptorBufferInfo outInfo{outputBuf_.getBuffer(), 0, VK_WHOLE_SIZE};

    VkWriteDescriptorSet writes[2]{};
    writes[0].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet          = descSet_;
    writes[0].dstBinding      = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[0].pBufferInfo     = &inInfo;

    writes[1]             = writes[0];
    writes[1].dstBinding  = 1;
    writes[1].pBufferInfo = &outInfo;

    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
}

} // namespace GSView
