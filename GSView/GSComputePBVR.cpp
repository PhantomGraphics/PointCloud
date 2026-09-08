#include "GSComputePBVR.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../PointCloud/GSPointCloud.h"
#include "../../CGLib/Volume/VolumeRenderer/PBVRPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

// Layout must match the GLSL shader structs exactly.
static_assert(sizeof(Phantom::PointCloud::GSPoint) == 68,
    "GSPoint layout changed — update gs_pbvr_gen.comp accordingly");
static_assert(sizeof(Phantom::Volume::PBVRVertex) == 28,
    "PBVRVertex layout changed — update gs_pbvr_gen.comp accordingly");
static_assert(offsetof(Phantom::Volume::PBVRVertex, color) == 12,
    "PBVRVertex color offset changed — update gs_pbvr_gen.comp accordingly");

namespace GSView {

namespace {

// Per-splat particle count. Must stay in lock-step with gs_pbvr_gen.comp:
//   Ni = clamp(round(densityScale * sigmoid(opacity) * maxPPS), 0, maxPPS)
// Note the lower bound is 0 (Phase 0): a fully transparent splat produces no
// particles instead of being forced to emit one.
static uint32_t particleCountForSplat(const Phantom::PointCloud::GSPoint& p,
                                      float densityScale, uint32_t maxPPS)
{
    const float op = 1.0f / (1.0f + std::exp(-p.opacity));
    const long  ni = std::lround(densityScale * op * static_cast<float>(maxPPS));
    if (ni <= 0) return 0u;
    return std::min<uint32_t>(static_cast<uint32_t>(ni), maxPPS);
}

} // namespace

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
    capacity_         = 0;
    generatedCount_   = 0;
    drawCount_        = 0;
    cachedNumSplats_  = 0;
    cachedGeneration_ = 0;
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
        capacity_ = generatedCount_ = drawCount_ = 0;
        return;
    }

    const uint32_t numSplats = static_cast<uint32_t>(cloud.points.size());

    // Rebuild the input SSBO whenever the splat count OR the data generation changes.
    // Keying on generation alone would suffice, but the count guard keeps the first
    // dispatch after create() working for callers that reuse a cloud with generation 0.
    if (numSplats != cachedNumSplats_ || cloud.generation != cachedGeneration_) {
        rebuildInputBuf(ctx, pool, cloud);
        cachedNumSplats_  = numSplats;
        cachedGeneration_ = cloud.generation;
    }

    // Rebuild output buffer when capacity is insufficient
    const uint32_t neededVerts = numSplats * maxPPS_;
    const VkDeviceSize neededBytes =
        static_cast<VkDeviceSize>(neededVerts) * sizeof(Phantom::Volume::PBVRVertex);
    if (!outputBuf_.isValid() || outputBuf_.getSize() < neededBytes) {
        rebuildOutputBuf(ctx, pool, numSplats);
    }

    capacity_ = neededVerts;

    // Mirror the shader's per-splat particle-count formula on the CPU so the UI and
    // scenario commands can report the real generated count (0 when all transparent).
    uint64_t generated = 0;
    for (const auto& p : cloud.points)
        generated += particleCountForSplat(p, densityScale_, maxPPS_);
    generatedCount_ = static_cast<uint32_t>(std::min<uint64_t>(generated, capacity_));

    // Padding slots are culled in the vertex shader, so we still submit the whole
    // capacity — but skip the draw entirely when nothing was generated.
    drawCount_ = (generatedCount_ > 0) ? capacity_ : 0;

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
