#include "GaussianPointRenderer.h"
#include "GaussianPointMath.h"

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

// Pbvr3d bank reuse, ViewConditioned invalidation (PLAN_pbvr_gps_ensemble_lod.md
// Phase 4, "Proportional/Extinction の候補生成と ViewConditioned の視点依存採択を
// 分ける"). ViewConditioned's keep-probability targets a view-dependent on-screen
// density (gpsTarget ~ sqrt(det Sigma2d), which scales like 1/viewDepth^2 under a
// dolly); reusing a bank generated at a very different distance silently drifts
// from that target instead of resampling to it. Proportional/Extinction's
// candidate count (lambda) has no view dependence, so they are exempt from this
// check. 15% is an applied placeholder, not a measured threshold -- unverified
// pending real-data tuning, same status as the Phase 2 LOD controller's initial
// candidates.
constexpr float kViewConditionedBankDriftTolerance = 1.15f;

// Binding indices shared by the compute shaders (see gps_splat.comp / gps_resolve.comp).
enum : uint32_t {
    B_INPUT = 0, B_DEPTH = 1, B_COLOR = 2, B_ACCUM = 3, B_STATS = 4, B_PARAMS = 5, B_SHREST = 6, B_PREPARED = 7, B_SCAN = 8, B_PARTICLES = 9, B_WORK = 10, B_BANK = 11
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
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(ctx.getPhysicalDevice(), &properties);
    driverVersion_ = properties.driverVersion;
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
        ssbo(B_PREPARED, VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_SCAN, VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_PARTICLES, VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_WORK, VK_SHADER_STAGE_COMPUTE_BIT),
        ssbo(B_BANK, VK_SHADER_STAGE_COMPUTE_BIT),
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
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 12 * frames_ },
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
    splatCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(SplatPushConstants) };
    if (!splatPipe_.create(ctx, splatCfg)) { available_ = false; return; }

    Phantom::VKG::ComputePipelineConfig pbvrCfg{};
    pbvrCfg.compSpv = pbvrSpv;
    pbvrCfg.descriptorSetLayout = computeDsl_.get();
    pbvrCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(SplatPushConstants) };
    if (!pbvr3dPipe_.create(ctx, pbvrCfg)) { available_ = false; return; }

    auto scanCfg = splatCfg;
    scanCfg.compSpv = ::VKG::loadSPVRepo("shaders/gps_scan.comp.spv");
    scanCfg.pushConstantRange.size = sizeof(ScanConstants);
    auto compactCfg = splatCfg;
    compactCfg.compSpv = ::VKG::loadSPVRepo("shaders/gps_compact.comp.spv");
    compactCfg.pushConstantRange.size = sizeof(PushConstants);  // gps_compact.comp only reads `pass`
    if (scanCfg.compSpv.empty() || compactCfg.compSpv.empty() ||
        !scanPipe_.create(ctx, scanCfg) || !compactPipe_.create(ctx, compactCfg)) {
        available_ = false; return;
    }

    Phantom::VKG::ComputePipelineConfig resolveCfg{};
    resolveCfg.compSpv = resolveSpv;
    resolveCfg.descriptorSetLayout = computeDsl_.get();
    resolveCfg.pushConstantRange = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(ResolvePushConstants) };
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
    params_.pbvr3dMethod = std::clamp(params_.pbvr3dMethod, 0, 4);
    params_.compactPipeline = std::clamp(params_.compactPipeline, 0, 2);
    // Metropolis (Phase 5) runs a sequential Markov chain per splat in
    // GaussianPointRenderer's own per-splat loop (gps_pbvr3d.comp), never through
    // gps_compact.comp's per-particle-parallel path -- that path has no notion of
    // "the previous accepted sample" a chain needs. compactPipeline==0 ("primitive
    // replay") is the one value that routes generation through that per-splat loop
    // (gps_scan.comp's mode==3 sets work[4]=2, which is gps_pbvr3d.comp::main()'s
    // gate for it; compactPipeline==1 "automatic compact" and ==2 "force point
    // replay" -- a confusingly-named value, it actually forces gps_scan.comp's
    // mode==4, work[4]=1, still handled by gps_compact.comp -- both stay on the
    // per-particle-parallel path). Force it unconditionally whenever Metropolis is
    // selected; never a silent wrong-answer risk, just an enforced combination
    // (same spirit as clamping sppSide/shDegree above, not a soft warning like
    // pointBudget+LOD-preset).
    if (params_.pbvr3dMethod == static_cast<int>(Pbvr3dMethod::Metropolis))
        params_.compactPipeline = 0;
    params_.lodMode = std::clamp(params_.lodMode, 0, 2);
    params_.ensemblesPerFrame = std::clamp(params_.ensemblesPerFrame, 1, 8);
    params_.targetEnsembles = std::max(1, params_.targetEnsembles);
    params_.lodFrameBudgetLowMs = std::max(0.1f, params_.lodFrameBudgetLowMs);
    params_.lodFrameBudgetHighMs = std::max(params_.lodFrameBudgetLowMs, params_.lodFrameBudgetHighMs);
    spp_ = static_cast<uint32_t>(params_.sppSide * params_.sppSide);

    // Keep the adaptive controller's config in sync with the same two knobs Manual
    // uses directly (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 3) -- R cap and
    // convergence target -- plus the Adaptive-only time budget. Harmless to update
    // unconditionally: lodController_.advance() only reads this config when
    // lodMode == Adaptive, and setConfig() takes effect on the next advance() call.
    {
        EnsembleLodController::Config cfg = lodController_.config();
        cfg.rMax = static_cast<uint32_t>(params_.ensemblesPerFrame);
        cfg.targetMax = static_cast<uint32_t>(params_.targetEnsembles);
        cfg.frameBudgetLowMs = params_.lodFrameBudgetLowMs;
        cfg.frameBudgetHighMs = params_.lodFrameBudgetHighMs;
        lodController_.setConfig(cfg);
    }
}

void GaussianPointRenderer::resetAccumulation(bool hard)
{
    resetPending_ = frames_;
    ++ensembleEpoch_;
    framesSinceReset_ = 0;
    // Every history-invalidating change (camera, params, data, resize, lodMode
    // switch) routes through here, so this is the single point that needs to
    // tell the adaptive controller "motion happened" (docs/todo/PLAN_pbvr_gps_ensemble_lod.md
    // Phase 2). Harmless when LodMode != Adaptive -- the flag is only consumed
    // by EnsembleLodController::advance(), which recordCompute()/update() only
    // call in Adaptive mode.
    lodController_.notifyMotion();
    // Pbvr3d particle-bank reuse (Phase 4): only a hard reset invalidates the
    // world-space bank. update()'s camera-only path calls resetAccumulation(false)
    // instead, so bankBuiltEpoch_[slot] == desiredBankEpoch_ stays true and
    // recordCompute() can reproject the existing bank under the new camera
    // rather than resampling it.
    if (hard) ++desiredBankEpoch_;
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

    if (!createWorkBuffers(ctx, pool)) { available_ = false; return; }
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
    resetAccumulation();   // fresh buffers -> restart accumulation
}

void GaussianPointRenderer::destroyFrameBuffers(VkDevice device)
{
    for (uint32_t f = 0; f < kMaxFrames; ++f) {
        preparedBuf_[f].destroy(device);
        scanBuf_[f].destroy(device);
        particleBuf_[f].destroy(device);
        workBuf_[f].destroy(device);
        bankBuf_[f].destroy(device);
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

    VkDescriptorBufferInfo prep{ preparedBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo scan{ scanBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo points{ particleBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo work{ workBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkDescriptorBufferInfo bank{ bankBuf_[f].getBuffer(), 0, VK_WHOLE_SIZE };
    VkWriteDescriptorSet w[12]{};
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
    set(7, B_PREPARED, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &prep);
    set(8, B_SCAN, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &scan);
    set(9, B_PARTICLES, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &points);
    set(10, B_WORK, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &work);
    set(11, B_BANK, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, &bank);
    vkUpdateDescriptorSets(device, 12, w, 0, nullptr);
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

        glm::dvec3 center(0.0);
        for (const auto& point : cloud_->points) center += glm::dvec3(point.x, point.y, point.z);
        objectCenter_ = glm::vec3(center / double(cloud_->points.size()));
        if (!createWorkBuffers(ctx, pool)) { available_ = false; return; }
        cachedGeneration_ = cloud_->generation;
        for (uint32_t f = 0; f < frames_; ++f) writeComputeSet(ctx.getDevice(), f);
        resetAccumulation();
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

    // --- confirm a pending Pbvr3d bank generation (Phase 4) -----------------
    // workBuf_ is host-coherent; by the time this slot is next visited (one
    // real frame later, given double buffering) the GPU has finished the
    // dispatch that set bankPendingEpoch_[frameIndex] in recordCompute().
    if (bankPendingEpoch_[frameIndex] != 0) {
        const auto* work = static_cast<const uint32_t*>(workBuf_[frameIndex].getMapped());
        if (work && work[4] == 0u) bankBuiltEpoch_[frameIndex] = bankPendingEpoch_[frameIndex];
        bankPendingEpoch_[frameIndex] = 0;
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
        resetAccumulation();
    else
        budgetThin_ = previousBudgetThin;

    // --- adaptive ensemble LOD (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 2) ---
    // Runs once per displayed frame regardless of lodMode so dt tracking stays
    // continuous; only consumed by recordCompute() when lodMode == Adaptive.
    {
        const auto now = std::chrono::steady_clock::now();
        float dtMs = 16.7f;
        if (hasLastUpdateTime_)
            dtMs = std::chrono::duration<float, std::milli>(now - lastUpdateTime_).count();
        lastUpdateTime_ = now;
        hasLastUpdateTime_ = true;

        if (params_.lodMode == static_cast<int>(LodMode::Adaptive)) {
            const bool timestampsSupported = queryPool_ != VK_NULL_HANDLE && tsPeriodNs_ > 0.0f;
            adaptiveRequest_ = lodController_.advance(dtMs, lastTimings_[frameIndex].computeMs,
                                                       timestampsSupported, ensembleHistory_[frameIndex]);
        }
    }

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
    mix(params_.seed);
    mixFloat(params_.densityScale);
    mixFloat(params_.maxPointsPerSplat);
    mixFloat(params_.opacityCutoff);
    mixFloat(params_.lowPass);
    mixFloat(params_.nearZ);
    mixFloat(params_.footprintCullPx);
    mixFloat(params_.gamma);
    mixFloat(params_.basePointsPerSplat);
    mixFloat(params_.pointBudget);
    mix(params_.pbvrZoomRecalibration);
    mix(params_.pbvrDensityClamp);
    mix(params_.compactPipeline);
    mixFloat(params_.pbvrReferencePixelLength);
    mix(static_cast<std::uint64_t>(shDeg) * 7 + params_.tonemapMode);
    mix(static_cast<std::uint64_t>(params_.pbvr3dMethod) * 3
        + static_cast<std::uint64_t>(path_ == Path::Pbvr3d));
    // lodMode changes the RNG stream structure (ensembleSeed mixing turns on/off),
    // so it must restart accumulation. ensemblesPerFrame/targetEnsembles are
    // deliberately excluded: tuning the quality knobs should continue an
    // existing history rather than discard it (PLAN_pbvr_gps_ensemble_lod.md Phase 2).
    mix(static_cast<std::uint64_t>(params_.lodMode));
    mixFloat(params_.background.x);
    mixFloat(params_.background.y);
    mixFloat(params_.background.z);
    mixFloat(camera_.focalX);
    mixFloat(camera_.focalY);
    mixFloat(camera_.cx);
    mixFloat(camera_.cy);
    // camera_.camPos is deliberately NOT mixed into h -- it and camera_.view are
    // compared separately below so Pbvr3d bank reuse (Phase 4) can tell "only the
    // camera moved" apart from every other invalidating change (which must always
    // hard-reset, camera-only changes may soft-reset when reuse is eligible).
    const bool paramsChanged = (h != lastChangeHash_);
    const bool cameraChanged = (camera_.view != lastView_) || (camera_.camPos != lastCamPos_);
    if (paramsChanged || cameraChanged) {
        const bool cameraOnly = cameraChanged && !paramsChanged;
        const bool canSoftReset = cameraOnly && path_ == Path::Pbvr3d && params_.pbvrBankReuse &&
            !params_.pbvrZoomRecalibration && params_.compactPipeline == 1;
        resetAccumulation(!canSoftReset);
        lastChangeHash_ = h;
        lastView_ = camera_.view;
        lastCamPos_ = camera_.camPos;
    }

    const uint32_t resetAccum = (resetPending_ > 0) ? 1u : 0u;
    if (resetPending_ > 0) --resetPending_;
    if (resetAccum) ensembleHistory_[frameIndex] = 0;

    ParamsUBO ubo{};
    ubo.view = camera_.view;
    ubo.p0 = glm::vec4(camera_.focalX, camera_.focalY, camera_.cx, camera_.cy);
    ubo.p1 = glm::vec4(params_.nearZ, params_.lowPass, static_cast<float>(kShC0), params_.opacityCutoff);
    ubo.dims = glm::uvec4(extent_.width, extent_.height, spp_,
                          static_cast<uint32_t>(std::clamp(params_.sppSide, 1, 4)));
    ubo.ctrl = glm::uvec4(numSplats_, params_.seedMode == 0 ? params_.seed : frameCounter_ + params_.seed * 747796405u,
                          static_cast<uint32_t>(params_.seedMode != 0),
                          static_cast<uint32_t>(params_.countMode != 0));
    ubo.p2 = glm::vec4(params_.maxPointsPerSplat, std::max(0.0f, params_.footprintCullPx),
                       params_.densityScale,
                       params_.pbvrZoomRecalibration ? static_cast<float>(gpm::pixelDensityScale(
                           -(camera_.view * glm::vec4(objectCenter_, 1.0f)).z,
                           camera_.focalX, camera_.focalY, params_.pbvrReferencePixelLength,
                           params_.nearZ)) : 1.0f);
    // bg.w is otherwise unused (composite/resolve only read bg.rgb) -- reused to
    // carry the density-clamp flag into gps_pbvr3d.comp without growing the UBO.
    ubo.bg = glm::vec4(params_.background, params_.pbvrDensityClamp ? 1.0f : 0.0f);
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
    ++framesSinceReset_;
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

    // --- decide how many independent ensembles to dispatch this frame -------
    // (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 1-2). Off always runs exactly
    // one and always uses ensembleSeed 0, reproducing the pre-Phase-1 renderer's
    // RNG stream bit-for-bit. Manual takes R/target straight from Params;
    // Adaptive takes them from lodController_ via the request update() computed
    // this frame (Phase 2).
    const bool lodActive = params_.lodMode != static_cast<int>(LodMode::Off);
    const bool adaptive  = params_.lodMode == static_cast<int>(LodMode::Adaptive);
    const uint32_t requestedR = lodActive
        ? (adaptive ? std::clamp(adaptiveRequest_.ensemblesPerFrame, 1u, 8u)
                    : static_cast<uint32_t>(std::clamp(params_.ensemblesPerFrame, 1, 8)))
        : 1u;
    const uint32_t target = lodActive
        ? (adaptive ? std::max(1u, adaptiveRequest_.targetEnsembles)
                    : static_cast<uint32_t>(std::max(1, params_.targetEnsembles)))
        : 1u;
    uint32_t& history = ensembleHistory_[frameIndex];
    const bool isResetFrame = (history == 0);
    const uint32_t remaining = (target > history) ? (target - history) : 0u;
    uint32_t effectiveR = lodActive ? std::min(requestedR, remaining) : 1u;
    if (numSplats_ == 0) effectiveR = std::min(effectiveR, 1u);
    lastEffectiveR_[frameIndex] = effectiveR;

    if (effectiveR == 0) {
        // Converged: this slot already holds `target` independent samples --
        // skip all GPU work and keep displaying accumBuf as-is.
        ts(1, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        ts(4, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        return;
    }

    // Identifies one independent ensemble's RNG stream, unique per (epoch, slot,
    // cumulative index) so a resumed/target-increased slot never replays a
    // stream it already used (design principle 5 in the plan doc). 0 is
    // reserved for the legacy bit-identical stream and only ever produced when
    // !lodActive.
    auto ensembleSeedFor = [&](uint32_t globalIndex) -> uint32_t {
        uint32_t h = ensembleEpoch_ * 2654435761u + frameIndex * 40503u + globalIndex * 2246822519u + 1u;
        h ^= h >> 16; h *= 0x7feb352du;
        h ^= h >> 15; h *= 0x846ca68bu;
        h ^= h >> 16;
        return h;
    };

    // stats is an atomic-accumulate target for every pass; clearing it once and
    // letting every ensemble atomicAdd into it makes the reported counts a
    // total across the whole frame's ensembles rather than just the last one.
    vkCmdFillBuffer(cmd, stats, 0, VK_WHOLE_SIZE, 0u);
    VkBufferMemoryBarrier statsToCompute =
        bufBarrier(stats, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         0, 0, nullptr, 1, &statsToCompute, 0, nullptr);

    // Grid-stride splat: cap the dispatch so any splat count is valid.
    const uint32_t groups = std::min<uint32_t>((numSplats_ + 63u) / 64u, 65535u);

    // Pbvr3d particle-bank reuse (Phase 4, connected to ensemble-LOD iteration
    // in slice 3, docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 4). Eligibility
    // (bankReuseEligible) no longer requires effectiveR == 1: a camera-only soft
    // reset can coincide with Manual/Adaptive requesting R > 1 ensembles this
    // frame (e.g. Manual with ensemblesPerFrame=4 while continuously dragging
    // the camera -- every frame re-triggers a soft reset, so every frame is
    // reset-eligible regardless of R). Only ensemble i==0 of the loop below may
    // actually reproject the existing bank; ensembles i==1..effectiveR-1 always
    // run a genuine independent prepare/scan/compact/generate, exactly as they
    // would with reuse disabled. This keeps every reported "independent sample"
    // honest -- the plan's "同じ有限bankの繰り返しで標本数だけを増やさない":
    // the R-1 later ensembles never replay the bank i==0 reused, and each of
    // them overwrites bankBuf_[frameIndex] with a fresh candidate set, so the
    // *next* reuse-eligible frame reprojects a bank that is itself only one
    // frame old, not an ever-more-stale copy of the same one. A stationary frame
    // with no new resetAccumulation() call has isResetFrame==false and always
    // falls through to genuine independent draws for all R ensembles, so
    // Off/Manual/Adaptive's progressive refinement while the camera is NOT
    // moving is completely unaffected by this feature (design principle 4: one
    // ensemble's probability model persists while moving; independent
    // refinement resumes once settled).
    // ViewConditioned-only drift check (Phase 4 slice 2): its keep-probability
    // targets a view-depth-dependent on-screen density, so a bank generated at a
    // very different distance would silently drift from that target instead of
    // resampling to it. Proportional/Extinction's candidate count has no view
    // dependence and are exempt (design principle in PLAN_pbvr_gps_ensemble_lod.md
    // Phase 4: "Proportional/Extinction の候補生成と ViewConditioned の視点依存
    // 採択を分ける").
    const bool isViewConditioned = params_.pbvr3dMethod == static_cast<int>(Pbvr3dMethod::ViewConditioned);
    const float curViewDepth = -(camera_.view * glm::vec4(objectCenter_, 1.0f)).z;
    const bool bankDistanceOk = !isViewConditioned ||
        bankRefViewDepth_ <= 0.0f || curViewDepth <= 0.0f ||
        (curViewDepth / bankRefViewDepth_ >= 1.0f / kViewConditionedBankDriftTolerance &&
         curViewDepth / bankRefViewDepth_ <= kViewConditionedBankDriftTolerance);

    const bool bankReuseEligible = isResetFrame && path_ == Path::Pbvr3d && params_.pbvrBankReuse &&
        !params_.pbvrZoomRecalibration && params_.compactPipeline == 1 &&
        bankBuiltEpoch_[frameIndex] == desiredBankEpoch_ && bankDistanceOk;
    lastBankReused_[frameIndex] = bankReuseEligible && numSplats_ > 0;

    for (uint32_t i = 0; i < effectiveR; ++i) {
        const bool first = (i == 0);
        const bool last  = (i + 1 == effectiveR);
        const uint32_t ensembleSeed = lodActive ? ensembleSeedFor(history + i) : 0u;

        // 1. clear this ensemble's depth/colour subpixel buffers
        vkCmdFillBuffer(cmd, depth, 0, VK_WHOLE_SIZE, 0xFFFFFFFFu);
        vkCmdFillBuffer(cmd, color, 0, VK_WHOLE_SIZE, 0xFFFFFFFFu);
        VkBufferMemoryBarrier toCompute[2] = {
            bufBarrier(depth, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
            bufBarrier(color, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
        };
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 0, nullptr, 2, toCompute, 0, nullptr);
        if (first) ts(1, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

        // Only the first ensemble of a reset frame may reproject the existing
        // bank; every later ensemble in this same R-loop (i > 0) always draws a
        // genuine independent sample below, regenerating the bank it leaves
        // behind (see the bankReuseEligible comment above).
        const bool bankReuse = first && bankReuseEligible;
        if (numSplats_ > 0 && bankReuse) {
            // Pbvr3d bank reuse (Phase 4): skip prepare/scan/compact entirely and
            // reproject the existing world-space bank (gps_compact.comp's last
            // generate pass) under the current camera -- only projection + SH
            // colour are redone (gps_pbvr3d.comp passes 3/4). work[3]/bank[] are
            // read exactly as left by that earlier generate.
            const auto* workMapped = static_cast<const uint32_t*>(workBuf_[frameIndex].getMapped());
            const uint32_t bankCount = workMapped ? workMapped[3] : 0u;
            const uint32_t reprojectGroups = std::min<uint32_t>((bankCount + 63u) / 64u, 65535u);

            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pbvr3dPipe_.getPipeline());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pbvr3dPipe_.getLayout(),
                                    0, 1, &computeSets_[frameIndex], 0, nullptr);
            SplatPushConstants pcDepth{ 3, 0 };
            vkCmdPushConstants(cmd, pbvr3dPipe_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcDepth), &pcDepth);
            vkCmdDispatch(cmd, reprojectGroups, 1, 1);

            VkBufferMemoryBarrier depthRW = bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 0, 0, nullptr, 1, &depthRW, 0, nullptr);
            if (first) ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

            SplatPushConstants pcColor{ 4, 0 };
            vkCmdPushConstants(cmd, pbvr3dPipe_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcColor), &pcColor);
            vkCmdDispatch(cmd, reprojectGroups, 1, 1);

            VkBufferMemoryBarrier toResolve[2] = {
                bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_READ_BIT),
                bufBarrier(color, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT),
            };
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 0, 0, nullptr, 2, toResolve, 0, nullptr);
            if (first) ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        } else if (numSplats_ > 0) {
            const auto& splat = (path_ == Path::Pbvr3d) ? pbvr3dPipe_ : splatPipe_;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getPipeline());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getLayout(),
                                    0, 1, &computeSets_[frameIndex], 0, nullptr);

            auto computeBarrier = [&]() {
                VkMemoryBarrier barrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
                barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
                vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &barrier, 0, nullptr, 0, nullptr);
            };
            SplatPushConstants prepare{2, ensembleSeed};
            vkCmdPushConstants(cmd, splat.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(prepare), &prepare);
            vkCmdDispatch(cmd, groups, 1, 1);
            computeBarrier();
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, scanPipe_.getPipeline());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, scanPipe_.getLayout(),
                0, 1, &computeSets_[frameIndex], 0, nullptr);
            auto scanPass = [&](uint32_t mode, const ScanLevel& level) {
                ScanConstants pc{mode, level.base, level.size, level.parent};
                vkCmdPushConstants(cmd, scanPipe_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);
                vkCmdDispatch(cmd, std::min(65535u, (level.size + 255u) / 256u), 1, 1);
                computeBarrier();
            };
            for (const auto& level : scanLevels_) scanPass(0, level);
            for (size_t li = scanLevels_.size(); li > 1; --li) scanPass(1, scanLevels_[li-2]);
            scanPass(params_.compactPipeline == 0 ? 3u : params_.compactPipeline == 2 ? 4u : 2u,
                     {scanLevels_.back().parent, 1, 0});
            VkBufferMemoryBarrier indirect = bufBarrier(workBuf_[frameIndex].getBuffer(),
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_SHADER_READ_BIT);
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0, 0, nullptr, 1, &indirect, 0, nullptr);
            auto compactPass = [&](uint32_t pass) {
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, compactPipe_.getPipeline());
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, compactPipe_.getLayout(),
                    0, 1, &computeSets_[frameIndex], 0, nullptr);
                PushConstants pc{pass};
                vkCmdPushConstants(cmd, compactPipe_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);
                vkCmdDispatchIndirect(cmd, workBuf_[frameIndex].getBuffer(), 0);
            };
            compactPass(0);
            computeBarrier();
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getPipeline());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getLayout(),
                0, 1, &computeSets_[frameIndex], 0, nullptr);
            SplatPushConstants pcDepth{ 0, ensembleSeed };
            vkCmdPushConstants(cmd, splat.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcDepth), &pcDepth);
            vkCmdDispatch(cmd, groups, 1, 1);

            VkBufferMemoryBarrier depthRW = bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 0, 0, nullptr, 1, &depthRW, 0, nullptr);

            if (first) ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

            compactPass(1);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getPipeline());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, splat.getLayout(),
                0, 1, &computeSets_[frameIndex], 0, nullptr);
            SplatPushConstants pcColor{ 1, ensembleSeed };
            vkCmdPushConstants(cmd, splat.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcColor), &pcColor);
            vkCmdDispatch(cmd, groups, 1, 1);

            if (path_ == Path::Pbvr3d && params_.compactPipeline == 1) {
                bankPendingEpoch_[frameIndex] = desiredBankEpoch_;
                // Every generate dispatch recalibrates the bank to the CURRENT
                // camera, whether triggered by a hard reset or by the drift
                // fallback below finding the previous reference too stale --
                // always overwrite so a bank refreshed by drift-fallback is
                // immediately reuse-eligible again at its new distance instead
                // of being perpetually re-flagged against a stale reference.
                bankRefViewDepth_ = curViewDepth;
            }

            VkBufferMemoryBarrier toResolve[2] = {
                bufBarrier(depth, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_READ_BIT),
                bufBarrier(color, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT),
            };
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 0, 0, nullptr, 2, toResolve, 0, nullptr);
            if (first) ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        } else if (first) {
            ts(2, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
            ts(3, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        }

        // 4. resolve -- fold this ensemble into the progressive accumulator.
        // Only the very first ensemble of a freshly-reset slot may restart the
        // average; every later ensemble (this frame or a later one) accumulates.
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, resolvePipe_.getPipeline());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, resolvePipe_.getLayout(),
                                0, 1, &computeSets_[frameIndex], 0, nullptr);
        ResolvePushConstants resolvePc{ (first && isResetFrame) ? 1u : 0u };
        vkCmdPushConstants(cmd, resolvePipe_.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0,
                           sizeof(resolvePc), &resolvePc);
        vkCmdDispatch(cmd, (extent_.width + 7u) / 8u, (extent_.height + 7u) / 8u, 1);
        if (last) ts(4, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

        if (!last) {
            // Re-synchronize everything this ensemble touched before the next
            // ensemble's clear (depth/colour/stats) and prepare pass (prepared/
            // scan/particle/work) reuse the same buffers.
            VkBufferMemoryBarrier next[9] = {
                bufBarrier(accum, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
                bufBarrier(depth, VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT),
                bufBarrier(color, VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT),
                bufBarrier(stats, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT),
                bufBarrier(preparedBuf_[frameIndex].getBuffer(),
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT),
                bufBarrier(scanBuf_[frameIndex].getBuffer(),
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT),
                bufBarrier(particleBuf_[frameIndex].getBuffer(),
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT),
                bufBarrier(workBuf_[frameIndex].getBuffer(),
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT),
                // WRITE covers a regenerating ensemble (gps_compact.comp's
                // generate pass); READ covers ensemble i==0 having only
                // reprojected (bank reuse, slice 3) -- either way the next
                // ensemble's own generate pass must wait for this one to finish
                // touching bankBuf_[frameIndex] before writing it.
                bufBarrier(bankBuf_[frameIndex].getBuffer(),
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT),
            };
            vkCmdPipelineBarrier(cmd,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                0, 0, nullptr, 9, next, 0, nullptr);
        }
    }
    history += effectiveR;

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

uint64_t GaussianPointRenderer::bufferBytes() const
{
    uint64_t bytes = gsInput_.getSize() + shRest_.getSize() + readbackBuf_.getSize();
    for (uint32_t f = 0; f < frames_; ++f)
        bytes += depthBuf_[f].getSize() + colorBuf_[f].getSize() + accumBuf_[f].getSize()
               + statsBuf_[f].getSize() + paramsUbo_[f].getSize()
               + preparedBuf_[f].getSize() + scanBuf_[f].getSize() + particleBuf_[f].getSize() + workBuf_[f].getSize()
               + bankBuf_[f].getSize();
    return bytes;
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
    s.expectedCount = static_cast<uint32_t>(std::min<uint64_t>(
        ((uint64_t(p[6]) << 32u) | p[0]) / 256u, UINT32_MAX));
    s.generatedCount = p[1];
    s.activeSamples  = p[2];
    s.drawnPoints    = p[3];
    s.candidateCount = p[4];
    s.accumFrames    = p[5];
    s.shDegreeData   = shDegreeData_;
    const auto* work = static_cast<const uint32_t*>(workBuf_[f].getMapped());
    s.compactFallback = work ? work[4] : 0u;

    const Stats& t = lastTimings_[f];
    s.clearMs = t.clearMs;
    s.splatDepthMs = t.splatDepthMs;
    s.splatColorMs = t.splatColorMs;
    s.resolveMs = t.resolveMs;
    s.computeMs = t.computeMs;

    s.ensemblesThisFrame = lastEffectiveR_[f];
    s.displayedEnsembles = ensembleHistory_[f];
    s.epoch = ensembleEpoch_;
    s.requestedEnsemblesPerFrame = static_cast<uint32_t>(std::max(1, params_.ensemblesPerFrame));
    s.effectiveEnsemblesPerFrame = lastEffectiveR_[f];
    s.adaptiveState = static_cast<uint32_t>(lodController_.state());
    s.framesSinceReset = framesSinceReset_;
    s.bankReused = lastBankReused_[f];
    return s;
}

bool GaussianPointRenderer::createWorkBuffers(const Phantom::VKG::VulkanContext& ctx,
                                              const Phantom::VKG::VulkanCommandPool& pool)
{
    scanLevels_.clear();
    uint32_t n = std::max(numSplats_, 1u), base = 0;
    do {
        const uint32_t parent = base + n;
        scanLevels_.push_back({base, n, parent});
        n = (n + 255u) / 256u;
        base = parent;
    } while (n > 1u);
    scanWords_ = base + 1u;
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(ctx.getPhysicalDevice(), &props);
    const VkDeviceSize prepBytes = VkDeviceSize(std::max(numSplats_, 1u)) * 112u;
    if (prepBytes > props.limits.maxStorageBufferRange) return false;
    particleCapacity_ = std::min(8u * 1024u * 1024u, props.limits.maxStorageBufferRange / 16u);
    // Cloud uploads and resize already wait for older submissions before reaching here.
    for (uint32_t f = 0; f < frames_; ++f) {
        preparedBuf_[f].destroy(ctx.getDevice()); scanBuf_[f].destroy(ctx.getDevice());
        particleBuf_[f].destroy(ctx.getDevice()); workBuf_[f].destroy(ctx.getDevice());
        bankBuf_[f].destroy(ctx.getDevice());
        if (!preparedBuf_[f].create(ctx, pool, prepBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
            !scanBuf_[f].create(ctx, pool, VkDeviceSize(scanWords_)*4u, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
            !particleBuf_[f].create(ctx, pool, VkDeviceSize(particleCapacity_)*16u, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
            !bankBuf_[f].create(ctx, pool, VkDeviceSize(particleCapacity_)*16u, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
            !workBuf_[f].createMapped(ctx, 8u*4u, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT))
            return false;
        const uint32_t zero[8] = {};
        workBuf_[f].write(zero, sizeof(zero));
    }
    return true;
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
    scanPipe_.destroy(device);
    compactPipe_.destroy(device);
    splatPipe_.destroy(device);
    pbvr3dPipe_.destroy(device);
    resolvePipe_.destroy(device);
    descPool_.destroy(device);
    computeDsl_.destroy(device);
    compositeDsl_.destroy(device);
    available_ = false;
}

} // namespace GSView
