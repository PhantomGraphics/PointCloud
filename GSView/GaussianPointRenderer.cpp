#include "GaussianPointRenderer.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"
#include "../PointCloud/GSPointCloud.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <functional>
#include <utility>

// The gps_*.comp GsPoint struct is 17 tightly-packed floats -- must match GSPoint.
static_assert(sizeof(Phantom::PointCloud::GSPoint) == 68,
              "GSPoint layout changed -- update gps_splat.comp / gps_pbvr3d.comp");

namespace GSView {

namespace {
constexpr double kShC0 = 0.28209479177387814;

// Binding indices shared by the compute shaders (see gps_splat.comp / gps_resolve.comp).
enum : uint32_t {
    B_INPUT = 0, B_DEPTH = 1, B_COLOR = 2, B_ACCUM = 3, B_STATS = 4, B_PARAMS = 5, B_SHREST = 6
};

VkBufferMemoryBarrier bufBarrier(VkBuffer buf, VkAccessFlags src, VkAccessFlags dst)
{
    VkBufferMemoryBarrier b{ VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
    b.srcAccessMask = src;
    b.dstAccessMask = dst;
    b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.buffer = buf;
    b.offset = 0;
    b.size   = VK_WHOLE_SIZE;
    return b;
}
} // namespace

void GaussianPointRenderer::onInit(const Phantom::VKG::VulkanContext& ctx,
                                   const Phantom::VKG::VulkanCommandPool& pool,
                                   VkRenderPass renderPass, uint32_t framesInFlight)
{
    ctx_ = &ctx;
    pool_ = &pool;
    renderPass_ = renderPass;
    deviceName_ = ctx.getDeviceName();
    frames_ = std::min<uint32_t>(framesInFlight, kMaxFrames);
    VkDevice dev = ctx.getDevice();

    // --- compute descriptor set layout (bindings 0..5) ---
    auto ssbo = [](uint32_t bind, VkShaderStageFlags stage) {
        VkDescriptorSetLayoutBinding b{};
        b.binding = bind;
        b.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        b.descriptorCount = 1;
        b.stageFlags = stage;
        return b;
    };
    VkDescriptorSetLayoutBinding paramsBinding{};
    paramsBinding.binding = B_PARAMS;
    paramsBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    paramsBinding.descriptorCount = 1;
    paramsBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    computeDsl_.create(dev, {
        ssbo(B_INPUT,  VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_DEPTH,  VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_COLOR,  VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_ACCUM,  VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_STATS,  VK_SHADER_STAGE_COMPUTE_BIT),
        paramsBinding,
        ssbo(B_SHREST, VK_SHADER_STAGE_COMPUTE_BIT),
    });

    // --- composite descriptor set layout (resolvedBuf + params, fragment) ---
    VkDescriptorSetLayoutBinding cResolved = ssbo(0, VK_SHADER_STAGE_FRAGMENT_BIT);
    VkDescriptorSetLayoutBinding cParams{};
    cParams.binding = 1;
    cParams.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    cParams.descriptorCount = 1;
    cParams.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    compositeDsl_.create(dev, { cResolved, cParams });

    // --- descriptor pool ---
    std::vector<VkDescriptorPoolSize> sizes = {
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 7 * frames_ },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2 * frames_ },
    };
    descPool_.create(dev, sizes, 2 * frames_);
    for (uint32_t f = 0; f < frames_; ++f) {
        computeSets_[f]   = descPool_.allocateSets(dev, { computeDsl_.get() }).front();
        compositeSets_[f] = descPool_.allocateSets(dev, { compositeDsl_.get() }).front();
    }

    // --- pipelines ---
    auto splatSpv = ::VKG::loadSPVRepo("shaders/gps_splat.comp.spv");
    auto pbvrSpv = ::VKG::loadSPVRepo("shaders/gps_pbvr3d.comp.spv");
    auto resolveSpv = ::VKG::loadSPVRepo("shaders/gps_resolve.comp.spv");
    auto compVertSpv = ::VKG::loadSPVRepo("shaders/gps_composite.vert.spv");
    auto compFragSpv = ::VKG::loadSPVRepo("shaders/gps_composite.frag.spv");
    if (splatSpv.empty() || pbvrSpv.empty() || resolveSpv.empty() ||
        compVertSpv.empty() || compFragSpv.empty()) {
        std::fprintf(stderr, "[GaussianPoint] shader SPV missing -- mode disabled\n");
        available_ = false;
        return;
    }

    Phantom::VKG::ComputePipelineConfig splatCfg{};
    splatCfg.compSpv = splatSpv;
    splatCfg.descriptorSetLayout = computeDsl_.get();
    splatCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants) };
    if (!splatPipe_.create(ctx, splatCfg)) { available_ = false; return; }

    Phantom::VKG::ComputePipelineConfig pbvrCfg{};
    pbvrCfg.compSpv = pbvrSpv;
    pbvrCfg.descriptorSetLayout = computeDsl_.get();
    pbvrCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants) };
    if (!pbvr3dPipe_.create(ctx, pbvrCfg)) { available_ = false; return; }

    Phantom::VKG::ComputePipelineConfig resolveCfg{};
    resolveCfg.compSpv = resolveSpv;
    resolveCfg.descriptorSetLayout = computeDsl_.get();
    if (!resolvePipe_.create(ctx, resolveCfg)) { available_ = false; return; }

    Phantom::VKG::PipelineConfig compCfg{};
    compCfg.vertSpv = compVertSpv;
    compCfg.fragSpv = compFragSpv;
    compCfg.descriptorSetLayout = compositeDsl_.get();
    compCfg.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    compCfg.cullMode = VK_CULL_MODE_NONE;
    compCfg.depthTest = false;
    compCfg.depthWrite = false;
    if (!compositePipe_.create(ctx, renderPass, compCfg)) { available_ = false; return; }

    // GPU timestamp query pool (optional -- a null period just leaves timings 0).
    tsPeriodNs_ = ctx.getTimestampPeriodNs();
    if (tsPeriodNs_ > 0.0f) {
        VkQueryPoolCreateInfo qi{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
        qi.queryType = VK_QUERY_TYPE_TIMESTAMP;
        qi.queryCount = kMarks * frames_;
        if (vkCreateQueryPool(dev, &qi, nullptr, &queryPool_) != VK_SUCCESS)
            queryPool_ = VK_NULL_HANDLE;
    }

    available_ = true;
}

void GaussianPointRenderer::setParams(const Params& p)
{
    params_ = p;
    params_.sppSide = std::clamp(params_.sppSide, 1, 4);
    params_.shDegree = std::clamp(params_.shDegree, 0, 3);
    params_.tonemapMode = std::clamp(params_.tonemapMode, 0, 2);
    params_.pbvr3dMethod = std::clamp(params_.pbvr3dMethod, 0, 2);
    spp_ = static_cast<uint32_t>(params_.sppSide * params_.sppSide);
}

void GaussianPointRenderer::resetAccumulation()
{
    resetPending_ = frames_;
}

void GaussianPointRenderer::onResize(const Phantom::VKG::VulkanContext& ctx,
                                     const Phantom::VKG::VulkanCommandPool& pool,
                                     VkExtent2D extent)
{
    if (!available_) return;
    if (extent.width == 0 || extent.height == 0) return;

    VkDevice dev = ctx.getDevice();
    vkDeviceWaitIdle(dev); // resize only -- allowed to stall here, not per frame
    destroyFrameBuffers(dev);

    extent_ = extent;
    spp_ = static_cast<uint32_t>(std::clamp(params_.sppSide, 1, 4));
    spp_ *= spp_;

    const VkDeviceSize pixels = static_cast<VkDeviceSize>(extent.width) * extent.height;
    const VkDeviceSize subSamples = pixels * spp_;

    if (!shRest_.isValid()) {
        const float dummy = 0.0f;
        shRest_.create(ctx, pool, sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, &dummy);
    }

    for (uint32_t f = 0; f < frames_; ++f) {
        depthBuf_[f].create(ctx, pool, subSamples * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        colorBuf_[f].create(ctx, pool, subSamples * sizeof(uint32_t),       // logarithmic HDR RGB
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        accumBuf_[f].create(ctx, pool, pixels * 4 * sizeof(float),          // vec4 (rgb sum, count)
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        statsBuf_[f].createMapped(ctx, 8 * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        if (!paramsUbo_[f].isValid())
            paramsUbo_[f].createMapped(ctx, sizeof(ParamsUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        writeComputeSet(dev, f);
        writeCompositeSet(dev, f);
    }
    resetPending_ = frames_;   // fresh buffers -> restart accumulation
}

void GaussianPointRenderer::destroyFrameBuffers(VkDevice device)
{
    for (uint32_t f = 0; f < kMaxFrames; ++f) {
        depthBuf_[f].destroy(device);
        colorBuf_[f].destroy(device);
        accumBuf_[f].destroy(device);
        statsBuf_[f].destroy(device);
    }
}

void GaussianPointRenderer::writeComputeSet(VkDevice device, uint32_t f)
{
    VkDescriptorBufferInfo in{ gsInput_.isValid() ? gsInput_.getBuffer() : accumBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo dp{ depthBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo cl{ colorBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo ac{ accumBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo st{ statsBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo pa{ paramsUbo_[f].getBuffer(), 0, sizeof(ParamsUBO) };
    VkDescriptorBufferInfo sh{ shRest_.getBuffer(), 0, VK_WHOLE_SIZE };

    VkWriteDescriptorSet w[7]{};
    auto set = [&](int i, uint32_t bind, VkDescriptorType type, const VkDescriptorBufferInfo* bi) {
        w[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[i].dstSet = computeSets_[f];
        w[i].dstBinding = bind;
        w[i].descriptorCount = 1;
        w[i].descriptorType = type;
        w[i].pBufferInfo = bi;
    };
    set(0, B_INPUT,  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &in);
    set(1, B_DEPTH,  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &dp);
    set(2, B_COLOR,  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &cl);
    set(3, B_ACCUM,  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &ac);
    set(4, B_STATS,  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &st);
    set(5, B_PARAMS, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, &pa);
    set(6, B_SHREST, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &sh);
    vkUpdateDescriptorSets(device, 7, w, 0, nullptr);
}

void GaussianPointRenderer::writeCompositeSet(VkDevice device, uint32_t f)
{
    VkDescriptorBufferInfo rs{ accumBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo pa{ paramsUbo_[f].getBuffer(),   0, sizeof(ParamsUBO) };
    VkWriteDescriptorSet w[2]{};
    w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[0].dstSet = compositeSets_[f];
    w[0].dstBinding = 0;
    w[0].descriptorCount = 1;
    w[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w[0].pBufferInfo = &rs;
    w[1] = w[0];
    w[1].dstBinding = 1;
    w[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    w[1].pBufferInfo = &pa;
    vkUpdateDescriptorSets(device, 2, w, 0, nullptr);
}

void GaussianPointRenderer::update(const Phantom::VKG::VulkanContext& ctx,
                                   const Phantom::VKG::VulkanCommandPool& pool,
                                   uint32_t frameIndex)
{
    if (!available_) return;

    // Upload the input + SH SSBOs when the cloud changes (rare -- on load).
    const bool haveCloud = cloud_ && !cloud_->points.empty();
    if (haveCloud && cloud_->generation != cachedGeneration_) {
        // Build replacements first. VulkanBuffer::create(initialData) submits the
        // staging copy and waits for the graphics queue, so all older frames have
        // stopped using the current buffers before they are destroyed below.
        Phantom::VKG::VulkanBuffer nextInput;
        Phantom::VKG::VulkanBuffer nextShRest;
        if (!nextInput.create(ctx, pool,
            static_cast<VkDeviceSize>(cloud_->points.size()) * sizeof(Phantom::PointCloud::GSPoint),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, cloud_->points.data())) {
            return;
        }
        if (!cloud_->shRest.empty()) {
            if (!nextShRest.create(ctx, pool, cloud_->shRest.size() * sizeof(float),
                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, cloud_->shRest.data())) {
                nextInput.destroy(ctx.getDevice());
                return;
            }
        } else {
            const float dummy = 0.0f;
            if (!nextShRest.create(ctx, pool, sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, &dummy)) {
                nextInput.destroy(ctx.getDevice());
                return;
            }
        }

        gsInput_.destroy(ctx.getDevice());
        shRest_.destroy(ctx.getDevice());
        gsInput_ = std::move(nextInput);
        shRest_ = std::move(nextShRest);
        numSplats_ = static_cast<uint32_t>(cloud_->points.size());
        shDegreeData_ = cloud_->shDegree;

        cachedGeneration_ = cloud_->generation;
        for (uint32_t f = 0; f < frames_; ++f) writeComputeSet(ctx.getDevice(), f);
        resetPending_ = frames_;
    } else if (!haveCloud) {
        numSplats_ = 0;
        shDegreeData_ = 0;
    }

    if (extent_.width == 0) return;

    // --- read this frame slot's GPU timestamps (2 frames old, fence already waited) ---
    if (queryPool_ && tsPeriodNs_ > 0.0f && tsWritten_[frameIndex]) {
        std::uint64_t marks[kMarks] = {};
        const VkResult r = vkGetQueryPoolResults(
            ctx.getDevice(), queryPool_, frameIndex * kMarks, kMarks,
            sizeof(marks), marks, sizeof(std::uint64_t), VK_QUERY_RESULT_64_BIT);
        if (r == VK_SUCCESS) {
            Stats& t = lastTimings_[frameIndex];
            const double toMs = static_cast<double>(tsPeriodNs_) * 1e-6;
            t.clearMs      = static_cast<float>((marks[1] - marks[0]) * toMs);
            t.splatDepthMs = static_cast<float>((marks[2] - marks[1]) * toMs);
            t.splatColorMs = static_cast<float>((marks[3] - marks[2]) * toMs);
            t.resolveMs    = static_cast<float>((marks[4] - marks[3]) * toMs);
            t.computeMs    = static_cast<float>((marks[4] - marks[0]) * toMs);
        }
    }

    // --- adaptive stochastic thinning to a point budget ---
    const double previousBudgetThin = budgetThin_;
    if (params_.pointBudget > 0.0f) {
        const std::uint32_t lastGen = getStats().generatedCount;
        if (lastGen > 0) {
            const double target = std::clamp(
                budgetThin_ * static_cast<double>(params_.pointBudget) / lastGen, 0.02, 1.0);
            budgetThin_ = 0.6 * budgetThin_ + 0.4 * target;   // damped
        }
    } else {
        budgetThin_ = 1.0;
    }
    const double budgetDelta = std::abs(budgetThin_ - previousBudgetThin);
    if (budgetDelta > std::max(1.0e-4, std::abs(previousBudgetThin) * 5.0e-3))
        resetPending_ = frames_;
    else
        budgetThin_ = previousBudgetThin;

    const int shDeg = std::min(std::clamp(params_.shDegree, 0, 3), shDegreeData_);

    // Restart the progressive accumulation when the camera or a render parameter
    // that changes the image moves.
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t v) { h = (h ^ v) * 1099511628211ull; };
    auto mixFloat = [&](float v) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof(bits));
        mix(bits);
    };
    mix(static_cast<std::uint64_t>(params_.sppSide) * 131 + params_.seedMode * 17 + params_.countMode);
    mixFloat(params_.densityScale);
    mixFloat(params_.maxPointsPerSplat);
    mixFloat(params_.opacityCutoff);
    mixFloat(params_.lowPass);
    mixFloat(params_.nearZ);
    mixFloat(params_.footprintCullPx);
    mixFloat(params_.gamma);
    mixFloat(params_.basePointsPerSplat);
    mixFloat(params_.pointBudget);
    mix(static_cast<std::uint64_t>(shDeg) * 7 + params_.tonemapMode);
    mix(static_cast<std::uint64_t>(params_.pbvr3dMethod) * 3
        + static_cast<std::uint64_t>(path_ == Path::Pbvr3d));
    mixFloat(params_.background.x);
    mixFloat(params_.background.y);
    mixFloat(params_.background.z);
    mixFloat(camera_.focalX);
    mixFloat(camera_.focalY);
    mixFloat(camera_.cx);
    mixFloat(camera_.cy);
    mixFloat(camera_.camPos.x);
    mixFloat(camera_.camPos.y);
    mixFloat(camera_.camPos.z);
    if (h != lastChangeHash_ || camera_.view != lastView_) {
        resetPending_ = frames_;
        lastChangeHash_ = h;
        lastView_ = camera_.view;
    }

    const uint32_t resetAccum = (resetPending_ > 0) ? 1u : 0u;
    if (resetPending_ > 0) --resetPending_;

    ParamsUBO ubo{};
    ubo.view = camera_.view;
    ubo.p0 = glm::vec4(camera_.focalX, camera_.focalY, camera_.cx, camera_.cy);
    ubo.p1 = glm::vec4(params_.nearZ, params_.lowPass, static_cast<float>(kShC0), params_.opacityCutoff);
    ubo.dims = glm::uvec4(extent_.width, extent_.height, spp_,
                          static_cast<uint32_t>(std::clamp(params_.sppSide, 1, 4)));
    ubo.ctrl = glm::uvec4(numSplats_, frameCounter_,
                          static_cast<uint32_t>(params_.seedMode != 0),
                          static_cast<uint32_t>(params_.countMode != 0));
    ubo.p2 = glm::vec4(params_.maxPointsPerSplat, std::max(0.0f, params_.footprintCullPx),
                       params_.densityScale, 0.0f);
    ubo.bg = glm::vec4(params_.background, 0.0f);
    ubo.camPos = glm::vec4(camera_.camPos, 0.0f);
    ubo.ctrl2 = glm::uvec4(resetAccum, static_cast<uint32_t>(shDeg),
                           static_cast<uint32_t>(params_.tonemapMode),
                           static_cast<uint32_t>(params_.pbvr3dMethod));
    ubo.p3 = glm::vec4(std::max(0.01f, params_.gamma),
                       std::max(0.0f, params_.basePointsPerSplat),
                       static_cast<float>(budgetThin_),
                       static_cast<float>(Phantom::PointCloud::GSPointCloud::coeffsPerChannel(shDegreeData_)));
    paramsUbo_[frameIndex].write(&ubo, sizeof(ubo));

    lastFrameIndex_ = frameIndex;
    ++frameCounter_;
}

void GaussianPointRenderer::recordCompute(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!available_ || extent_.width == 0) return;

    const VkBuffer depth = depthBuf_[frameIndex].getBuffer();
    const VkBuffer color = colorBuf_[frameIndex].getBuffer();
    const VkBuffer stats = statsBuf_[frameIndex].getBuffer();
    const VkBuffer accum = accumBuf_[frameIndex].getBuffer();

    const uint32_t tsBase = frameIndex * kMarks;
    auto ts = [&](uint32_t mark, VkPipelineStageFlagBits stage) {
        if (queryPool_) vkCmdWriteTimestamp(cmd, stage, queryPool_, tsBase + mark);
    };
    if (queryPool_) { vkCmdResetQueryPool(cmd, queryPool_, tsBase, kMarks); tsWritten_[frameIndex] = true; }
    ts(0, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT);

    // 1. clear
    vkCmdFillBuffer(cmd, depth, 0, VK_WHOLE_SIZE, 0xFFFFFFFFu);
    vkCmdFillBuffer(cmd, color, 0, VK_WHOLE_SIZE, 0xFFFFFFFFu);
    vkCmdFillBuffer(cmd, stats, 0, VK_WHOLE_SIZE, 0u);

    VkBufferMemoryBarrier toCompute[3] = {
        bufBarrier(depth, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
        bufBarrier(color, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
        bufBarrier(stats, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
    };
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         0, 0, nullptr, 3, toCompute, 0, nullptr);
    ts(1, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

    // Grid-stride splat: cap the dispatch so any splat count is valid.
    const uint32_t groups = std::min<uint32_t>((numSplats_ + 63u) / 64u, 65535u);

    if (numSplats_ > 0) {
        const auto& splat = (path_ == Path::Pbvr3d) ? pbvr3dPipe_ : splatPipe_;
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getPipeline());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getLayout(),
                                0, 1, &computeSets_[frameIndex], 0, nullptr);

        PushConstants pcDepth{ 0 };
        vkCmdPushConstants(cmd, splat.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcDepth), &pcDepth);
        vkCmdDispatch(cmd, groups, 1, 1);

        VkBufferMemoryBarrier depthRW = bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 0, nullptr, 1, &depthRW, 0, nullptr);

        ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

        PushConstants pcColor{ 1 };
        vkCmdPushConstants(cmd, splat.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcColor), &pcColor);
        vkCmdDispatch(cmd, groups, 1, 1);

        VkBufferMemoryBarrier toResolve[2] = {
            bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_READ_BIT),
            bufBarrier(color, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT),
        };
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 0, nullptr, 2, toResolve, 0, nullptr);
        ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
    } else {
        ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
    }

    // 4. resolve
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, resolvePipe_.getPipeline());
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, resolvePipe_.getLayout(),
                            0, 1, &computeSets_[frameIndex], 0, nullptr);
    vkCmdDispatch(cmd, (extent_.width + 7u) / 8u, (extent_.height + 7u) / 8u, 1);
    ts(4, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

    (void)stats;
    VkBufferMemoryBarrier toFrag =
        bufBarrier(accum, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                         0, 0, nullptr, 1, &toFrag, 0, nullptr);
}

void GaussianPointRenderer::recordComposite(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!available_ || extent_.width == 0) return;
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipe_.getPipeline());
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipe_.getLayout(),
                            0, 1, &compositeSets_[frameIndex], 0, nullptr);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

bool GaussianPointRenderer::readAccumulationLinear(std::vector<glm::vec4>& out)
{
    if (!available_ || !ctx_ || !pool_ || extent_.width == 0 || frames_ == 0) return false;
    const VkDeviceSize bytes = static_cast<VkDeviceSize>(extent_.width) * extent_.height * sizeof(glm::vec4);
    if (!readbackBuf_.isValid() || readbackBuf_.getSize() != bytes) {
        readbackBuf_.destroy(ctx_->getDevice());
        if (!readbackBuf_.createMapped(*ctx_, bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT)) return false;
    }

    const uint32_t f = (lastFrameIndex_ + 1u) % frames_;
    VkCommandBuffer cmd = pool_->beginSingleTimeCommands();
    VkBufferMemoryBarrier toCopy =
        bufBarrier(accumBuf_[f].getBuffer(), VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         0, 0, nullptr, 1, &toCopy, 0, nullptr);
    VkBufferCopy copy{ 0, 0, bytes };
    vkCmdCopyBuffer(cmd, accumBuf_[f].getBuffer(), readbackBuf_.getBuffer(), 1, &copy);
    pool_->endSingleTimeCommands(cmd);

    const void* mapped = readbackBuf_.getMapped();
    if (!mapped) return false;
    out.resize(static_cast<size_t>(extent_.width) * extent_.height);
    std::memcpy(out.data(), mapped, static_cast<size_t>(bytes));
    return true;
}

GaussianPointRenderer::Stats GaussianPointRenderer::getStats() const
{
    Stats s;
    if (!available_) return s;
    // Read the frame buffer NOT being written this frame; its GPU work finished
    // at least one frame ago (host-coherent mapped memory, so no invalidate).
    const uint32_t f = (lastFrameIndex_ + 1u) % frames_;
    const auto* p = static_cast<const uint32_t*>(statsBuf_[f].getMapped());
    if (!p) return s;
    s.expectedCount  = p[0] / 256u;
    s.generatedCount = p[1];
    s.activeSamples  = p[2];
    s.drawnPoints    = p[3];
    s.candidateCount = p[4];
    s.accumFrames    = p[5];
    s.shDegreeData   = shDegreeData_;

    const Stats& t = lastTimings_[f];
    s.clearMs = t.clearMs;
    s.splatDepthMs = t.splatDepthMs;
    s.splatColorMs = t.splatColorMs;
    s.resolveMs = t.resolveMs;
    s.computeMs = t.computeMs;
    return s;
}

void GaussianPointRenderer::onCleanup(VkDevice device)
{
    destroyFrameBuffers(device);
    for (uint32_t f = 0; f < kMaxFrames; ++f)
        paramsUbo_[f].destroy(device);
    if (queryPool_) { vkDestroyQueryPool(device, queryPool_, nullptr); queryPool_ = VK_NULL_HANDLE; }
    gsInput_.destroy(device);
    shRest_.destroy(device);
    readbackBuf_.destroy(device);
    compositePipe_.destroy(device);
    splatPipe_.destroy(device);
    pbvr3dPipe_.destroy(device);
    resolvePipe_.destroy(device);
    descPool_.destroy(device);
    computeDsl_.destroy(device);
    compositeDsl_.destroy(device);
    available_ = false;
}

} // namespace GSView
