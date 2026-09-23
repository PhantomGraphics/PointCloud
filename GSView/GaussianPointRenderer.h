#pragma once

// -----------------------------------------------------------------------------
// GaussianPointRenderer -- GPS-style screen-space Gaussian-Point renderer
// (docs/todo/PLAN_gsview_gaussian_point_pbvr.md Phase 2, MVP).
//
// Per frame, entirely on the GPU, no CPU wait / vkDeviceWaitIdle:
//   1. clear the per-subpixel depth/colour/stats buffers (vkCmdFillBuffer)
//   2. gps_splat/gps_pbvr3d pass 2 -- prepare/count once per Gaussian
//   3. gps_scan -- hierarchical exclusive scan of variable particle counts
//   4. gps_compact -- point-parallel generation into packed candidate slots,
//      nearest depth, then colour selection from cached samples
//   4. gps_resolve.comp       -- average subpixels and update progressive history
//   5. gps_composite (graphics, inside the swapchain pass) -- blit to screen
//
// Current scope includes SH degree 0..3, temporal accumulation, conservative
// frustum/footprint culling, point-budget thinning, and the PBVR3D comparison.
// Large outputs replay point-parallel samples for colour instead of storing
// every candidate; no particles are dropped. Hierarchical occlusion is future work.
// -----------------------------------------------------------------------------

#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanComputePipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanPipeline.h"
#include "../../CGLib/VulkanGraphics/VulkanDescriptorPool.h"
#include "EnsembleLodController.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace Phantom::VKG { class VulkanContext; class VulkanCommandPool; }
namespace Phantom::PointCloud { struct GSPointCloud; }

namespace GSView {

class GaussianPointRenderer {
public:
    static constexpr uint32_t kMaxFrames = 2;

    // Which splat pass runs. Both feed the same depth/colour/accum buffers and
    // the same resolve + composite, so the two paths are directly comparable
    // (docs/todo/PLAN_gsview_gaussian_point_pbvr.md Phase 4).
    //   GaussianPoint - GPS screen-space stochastic-opaque-point scatter.
    //   Pbvr3d        - object-space "3D Gaussian -> world particles" scatter.
    enum class Path { GaussianPoint, Pbvr3d };

    // Pbvr3d particle-count rule (research comparison, Phase 4).
    //   Proportional    - N ~ densityScale * sigmoid(opacity) * basePointsPerSplat  (view-independent)
    //   Extinction      - N ~ densityScale * -log(1-sigmoid(opacity)) * basePointsPerSplat (view-independent)
    //   ViewConditioned - generate an extinction-count candidate set, then thin per
    //                     particle so the on-screen count matches the GPS target
    //                     spp * 2*pi*sqrt(det Sigma2d) * Li2(o).
    //   Metropolis      - same target count (N) as Extinction, but positions are drawn by an
    //                     independence-sampler Metropolis chain (uniform proposal over a +/-3-sigma
    //                     whitened cube, standard Metropolis acceptance against the Gaussian's own
    //                     density) instead of direct closed-form sampling -- literature-confirmed
    //                     via kvs::CellByCellMetropolisSampling (docs/todo/PLAN_pbvr_gps_ensemble_lod.md
    //                     Phase 5). A research comparison only: Gaussians can already be sampled
    //                     directly/exactly, so this exists to compare against that, not to replace it.
    //                     Requires compactPipeline == 0 "primitive replay" (setParams() enforces
    //                     this): the chain's sequential state dependency across accepted samples is
    //                     incompatible with gps_compact.comp's per-particle-parallel path (used by
    //                     both compactPipeline==1 "automatic compact" and ==2 "force point replay" --
    //                     despite that name, 2 still ends up on gps_compact.comp), so this method
    //                     always runs through GaussianPointRenderer's own per-splat loop in
    //                     gps_pbvr3d.comp instead (the one compactPipeline value -- 0 -- that reaches
    //                     it, via gps_scan.comp's mode==3 setting work[4]=2).
    enum class Pbvr3dMethod { Proportional = 0, Extinction = 1, ViewConditioned = 2, Metropolis = 3, ViewConditionedGps2D = 4 };

    // Ensemble LOD mode (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 1).
    // An "ensemble" is one complete independent clear -> splat -> resolve pass,
    // folded into the progressive accumulator as one more independent sample set.
    //   Off      - exactly one ensemble per displayed frame (legacy behaviour, bit-identical RNG stream).
    //   Manual   - up to `ensemblesPerFrame` ensembles per displayed frame, capped at `targetEnsembles`
    //              cumulative samples in the slot's current history.
    //   Adaptive - `ensemblesPerFrame`/`targetEnsembles` set `lodController_`'s R cap and
    //              convergence target (Phase 3: the same two knobs Manual uses directly,
    //              reinterpreted as an upper bound here); the actual per-frame R comes from
    //              `lodController_` (EnsembleLodController, Phase 2): R=1 while the
    //              camera/params/data are moving or still settling, then ramps up within
    //              `lodFrameBudgetLowMs`/`lodFrameBudgetHighMs` once quiet, and stops once
    //              the target is reached.
    enum class LodMode { Off = 0, Manual = 1, Adaptive = 2 };

    struct Params {
        int   sppSide          = 2;      // subpixel grid side (spp = sppSide^2), 1..4
        int   seedMode         = 1;      // 0 = deterministic, 1 = frame-varying
        uint32_t seed          = 0;      // independent reproducible random stream
        int   countMode        = 0;      // 0 = Poisson, 1 = stochastic rounding
        float densityScale     = 1.0f;
        float maxPointsPerSplat = 2048.f;
        glm::vec3 background    = glm::vec3(0.05f);
        float opacityCutoff    = 1.0e-3f;
        float lowPass          = 0.3f;
        float nearZ            = 0.05f;
        int   shDegree         = 0;      // clamped to the loaded data's degree
        int   tonemapMode      = 0;      // 0 none/clamp, 1 Reinhard, 2 ACES
        float gamma            = 1.0f;   // display gamma applied at composite
        float footprintCullPx  = 0.4f;   // cull Gaussians whose projected sigma is below this
        int   pbvr3dMethod     = 0;      // Pbvr3dMethod
        float basePointsPerSplat = 512.f; // Pbvr3d base count knob
        bool pbvrZoomRecalibration = false; // opt-in; reference pixel length is explicit
        float pbvrReferencePixelLength = 0.01f;
        int compactPipeline = 1; // 0 primitive replay, 1 automatic compact, 2 force point replay
        float pointBudget      = 0.f;    // 0 = unlimited; else adaptive stochastic thinning
        int   lodMode           = 0;     // LodMode
        int   ensemblesPerFrame = 1;     // R (Manual) / R cap (Adaptive), clamped to 1..8 (Off always runs 1)
        int   targetEnsembles   = 1;     // cumulative independent samples a slot's history stops refining at
        // Adaptive-only (Phase 3): EnsembleLodController's GPU time budget, in ms. Ramps R
        // up while under lodFrameBudgetLowMs, down once over lodFrameBudgetHighMs.
        float lodFrameBudgetLowMs  = 16.7f;
        float lodFrameBudgetHighMs = 33.3f;
        // Pbvr3d-only, opt-in (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 4, default off
        // = zero behaviour change). When true, a camera-only change (same data/params, only
        // the view matrix moved) reuses the last generated ensemble's world-space particle
        // bank instead of resampling it -- only reprojection + SH colour are redone. This
        // applies to at most the FIRST ensemble of that reset frame; if ensemble-LOD requests
        // R>1 for that same frame (e.g. Manual with ensemblesPerFrame>1 during continuous
        // camera drag), ensembles 2..R still draw genuine independent samples and overwrite
        // the bank, so the same finite bank is never replayed to inflate the reported sample
        // count. Ineligible combinations (pbvrZoomRecalibration on, compactPipeline !=
        // automatic-compact, no bank built yet, ViewConditioned bank drift) silently fall back
        // to full regeneration for the whole frame; never a correctness issue, only a missed
        // speedup. See GaussianPointRenderer::recordCompute().
        bool pbvrBankReuse = false;
        // Extinction/ViewConditioned candidate-count clamp (docs/todo/PLAN_pbvr_gps_ensemble_lod.md
        // Phase 0, literature-confirmed via kvs::CellByCellSampling::ParticleDensityMap -- the JCST
        // 2010 paper's own co-author's reference implementation). The extinction density formula
        // -log(1-opacity) is unbounded as opacity->1; the reference implementation clamps the
        // resulting per-volume density at max_density = 1/pixelLength^3 (pixelLength = object-space
        // size of one screen pixel at the splat's depth), which is algebraically equivalent to
        // capping the candidate count itself at splatVolume/pixelLength^3 -- at most ~1 candidate per
        // pixel-footprint-sized volume of the splat's own world-space extent. Proportional (method 0)
        // is exempt: its op-bounded formula never diverges, matching the literature's density formula
        // only needing this clamp for the extinction branch. Opt-in (default off = zero behaviour
        // change): scenes/configs already calibrated against the uncapped formula would see fewer
        // candidates once this clamps them -- a deliberate, literature-accurate density change, not a
        // bug fix to apply silently. Unlike pbvrBankReuse/pbvrZoomRecalibration this changes the
        // rendered image, so toggling it must invalidate accumulation (see update()'s hash mixing).
        bool pbvrDensityClamp = false;
        // Footprint calibration of the Pbvr3d particle count (docs/todo/
        // PLAN_footprint_aware_density_calibration.md Phase 3; theory in
        // docs/paper/NOTE_footprint_density_calibration.md). 0 None (C0, shipped rule),
        // 1 ObjectZoom (C1 -- the same as pbvrZoomRecalibration, which stays as the
        // legacy switch for C1 and only applies while this is 0), 2 PerSplatDepth (C2:
        // baseK * l0^2 fx fy / z_i^2), 3 PerSplatFootprint (C3: spp * 2*pi*sqrt(det
        // Sigma2d) * g(o), absolute -- basePointsPerSplat is ignored). ViewConditioned
        // keeps its own GPS-targeted thinning and ignores the level.
        int pbvrFootprintCalibration = 0;
        // C3+R: with Extinction at level 3, add the low-pass jitter and keep each
        // particle with GaussianPointMath::radialKeepProbability so the projected
        // intensity is the GPS one. Ignored for other method/level combinations.
        bool pbvrRadialCorrection = false;
        // Give every Pbvr3d particle its splat-centre depth (the GPS depth rule) instead
        // of its own; isolates the depth-rule residual (NOTE Sec. 5.2).
        bool pbvrCentreDepth = false;
        // Variable-footprint GPS points (level F, PLAN_footprint_aware_density_calibration.md
        // Phase 4): every point covers an s x s block of subpixels and the expected count is
        // divided by s^2. 1 = the GPS point (bit-identical). gpAdaptiveFootprintKappa > 0
        // instead picks s per splat as clamp(floor(kappa * sigmaMin), 1, gpFootprintMax)
        // with sigmaMin the projected minor-axis std-dev in subpixels. gpFootprintCompensate
        // shrinks the sampling covariance by the block variance (s/side)^2/12. Applies to
        // gps_splat.comp (GaussianPoint and ViewConditioned 2D).
        int   gpPointFootprint = 1;
        float gpAdaptiveFootprintKappa = 0.0f;
        int   gpFootprintMax = 4;
        bool  gpFootprintCompensate = false;
    };

    // The calibration level actually applied: pbvrFootprintCalibration, or C1 when
    // only the legacy pbvrZoomRecalibration switch is set.
    static int effectiveCalibrationLevel(const Params& p) {
        if (p.pbvrFootprintCalibration != 0) return p.pbvrFootprintCalibration;
        return p.pbvrZoomRecalibration ? 1 : 0;
    }

    struct Camera {
        glm::mat4 view{ 1.0f };   // world -> camera (GLM: camera looks -Z)
        glm::vec3 camPos{ 0.0f }; // world-space camera position (for SH view dir)
        float focalX = 1.0f, focalY = 1.0f;   // pixels
        float cx = 0.0f, cy = 0.0f;           // principal point (pixels)
    };

    struct Stats {
        uint32_t expectedCount  = 0;   // sum of per-Gaussian E[N], rounded
        uint32_t generatedCount = 0;   // accepted/generated points before viewport clipping
        uint32_t activeSamples  = 0;   // covered subpixels after resolve
        uint32_t drawnPoints    = 0;   // points that landed on screen
        uint32_t candidateCount = 0;   // PBVR candidate points before view-conditioned thinning
        // ViewConditioned only: splats whose candidate count fell below the GPS target, so
        // keepProb saturated at 1 and the splat is under-covered (summed over the frame's
        // ensembles; 0 on bank-reused ensembles, which skip the prepare pass).
        // docs/todo/PLAN_footprint_aware_density_calibration.md Phase 0.
        uint32_t keepSaturated  = 0;
        uint32_t accumFrames    = 0;   // sample sets in the displayed frame-slot accumulator
        uint32_t compactFallback = 0; // 0 cached compact, 1 bounded-memory replay, 2 primitive fallback
        int      shDegreeData   = 0;   // SH degree present in the loaded data
        // GPU pass times in ms (0 when timestamps are unsupported), lagged one frame.
        float    clearMs = 0.f, splatDepthMs = 0.f, splatColorMs = 0.f, resolveMs = 0.f, computeMs = 0.f;
        // Subdivision of splatDepthMs (first ensemble only; docs/todo/PLAN_footprint_aware_density_calibration.md
        // Phase 0): prepare (per-Gaussian count/projection), scan (exclusive scan), and the
        // remainder = compact depth pass (per-point sampling + depth atomics). 0 on bank-reuse frames.
        float    prepareMs = 0.f, scanMs = 0.f;

        // Ensemble LOD (Phase 1). displayedEnsembles is the CPU-tracked cumulative
        // independent-sample count of the displayed slot's current history --
        // authoritative (unlike accumFrames, it is not subject to the one-frame
        // GPU readback lag or double-buffering skew, docs/todo/PLAN_pbvr_gps_ensemble_lod.md Sec.2).
        uint32_t ensemblesThisFrame          = 0; // R actually dispatched this frame for the displayed slot
        uint32_t displayedEnsembles          = 0; // cumulative independent samples in the displayed slot's history
        uint32_t epoch                       = 0; // increments each time the sample history restarts
        uint32_t requestedEnsemblesPerFrame  = 0; // raw Params.ensemblesPerFrame
        uint32_t effectiveEnsemblesPerFrame  = 0; // == ensemblesThisFrame, kept alongside the request for clarity

        // Ensemble LOD (Phase 2). Only meaningful when lodMode == Adaptive; holds
        // the EnsembleLodController::State the displayed slot is in
        // (0=Moving, 1=Settling, 2=Refining, 3=Converged).
        uint32_t adaptiveState = 0;

        // Ensemble LOD (Phase 3). Frames of update() elapsed since the last
        // resetAccumulation() -- the "elapsed time" evaluation scripts need to
        // relate displayedEnsembles/adaptiveState to how long the scene has been
        // settled, without depending on wall-clock time (which varies by machine).
        uint32_t framesSinceReset = 0;

        // Pbvr3d particle-bank reuse (Phase 4). true when this frame's FIRST
        // ensemble reprojected the displayed slot's existing world-space bank
        // instead of resampling it (see Params::pbvrBankReuse). When ensemble-LOD
        // requests R>1 for this frame, any later ensembles (2..R) always drew a
        // genuine independent sample regardless of this flag. Always false
        // outside Pbvr3d.
        bool bankReused = false;
    };

    // Force the progressive accumulation to restart on the next frames. `hard`
    // also invalidates the Pbvr3d particle bank (Phase 4); pass false only from
    // the camera-only-change path in update(), which keeps the bank valid since
    // its world-space contents don't depend on the camera.
    void resetAccumulation(bool hard = true);

    void onInit(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight);
    void onCleanup(VkDevice device);

    // Recreate extent-dependent buffers. Call from the app's onSwapChainCreated
    // and once after onInit.
    void onResize(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                  VkExtent2D extent);

    void setGSCloud(const Phantom::PointCloud::GSPointCloud* cloud) { cloud_ = cloud; }

    // Measurement-only shader variant (docs/todo/PLAN_footprint_aware_density_calibration.md
    // Phase 0 cost breakdown). Bit flags: 1 = skip depth/colour atomics, 2 = replace the
    // GPS corrected-radius inverse (invDilog Newton) with a plain Gaussian radius. Renders a
    // WRONG image by design, so it is deliberately not a Params field (no UI, no hash, not
    // saved); 0 = normal rendering. Only reachable via the SetGpProfileVariant command.
    void setProfileVariant(uint32_t v) { profileVariant_ = v & 3u; }
    uint32_t getProfileVariant() const { return profileVariant_; }
    void setParams(const Params& p);
    void setCamera(const Camera& c) { camera_ = c; }
    void setPath(Path p) { if (p != path_) { path_ = p; resetAccumulation(); } }
    Path getPath() const { return path_; }

    bool isAvailable() const { return available_; }
    const char* backendName() const { return "32-bit two-pass"; }
    const std::string& deviceName() const { return deviceName_; }
    uint32_t driverVersion() const { return driverVersion_; }
    // Requested buffer bytes only; excludes allocator overhead and other renderers.
    uint64_t bufferBytes() const;
    Stats getStats() const;

    // Called from GSViewRenderer::onUpdate: uploads the input SSBO if the cloud
    // changed and writes the per-frame params UBO. No command recording.
    void update(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                uint32_t frameIndex);

    // Called from the app's onPreRender (before the swapchain render pass).
    void recordCompute(VkCommandBuffer cmd, uint32_t frameIndex);

    // Called from GSViewRenderer::onRender (inside the swapchain render pass).
    void recordComposite(VkCommandBuffer cmd, uint32_t frameIndex);

    // Synchronously copies the last completed frame-slot accumulator for
    // verification commands. This is never used on the normal render path.
    bool readAccumulationLinear(std::vector<glm::vec4>& out);

private:
    struct ParamsUBO {
        glm::mat4  view;
        glm::vec4  p0;    // focalX, focalY, cx, cy
        glm::vec4  p1;    // nearZ, lowPass, shC0, opacityCutoff
        glm::uvec4 dims;  // width, height, spp, sppSide
        glm::uvec4 ctrl;  // numSplats, frameIndex, seedMode, countMode
        glm::vec4  p2;    // maxPointsPerSplat, footprintCullPx, densityScale, 0
        glm::vec4  bg;    // background rgb, 0
        glm::vec4  camPos; // world-space camera position, 0
        glm::uvec4 ctrl2; // resetAccum, shDegree, tonemapMode, pbvr3dMethod
        glm::vec4  p3;    // gamma, basePointsPerSplat, budgetThin, SH storage stride/channel
        glm::uvec4 ctrl3; // footprint calibration level, radial correction, centre depth, footprint compensation
        glm::vec4  p4;    // referencePixelLength, footprint s, adaptive kappa, adaptive s max
    };
    struct PushConstants { uint32_t pass; };  // gps_compact.comp (pass only, unchanged layout)
    // gps_splat.comp / gps_pbvr3d.comp: ensembleSeed identifies one independent
    // ensemble's RNG stream within this frame's dispatch sequence. 0 reproduces
    // the pre-Phase-1 stream exactly (see the `pc.ensembleSeed != 0u` guard in
    // both shaders), so LodMode::Off and the first ensemble of every epoch stay
    // bit-identical to the legacy single-pass renderer.
    struct SplatPushConstants { uint32_t pass; uint32_t ensembleSeed; };
    struct ScanConstants { uint32_t mode, base, size, parent; };
    // gps_resolve.comp: whether THIS ensemble restarts the progressive average.
    // Moved out of ParamsUBO (Phase 1) because that UBO is written once per
    // update() call but a frame can now resolve several ensembles -- only the
    // first ensemble of a freshly-reset slot may restart the accumulator.
    struct ResolvePushConstants { uint32_t resetAccum; };

    bool available_ = false;
    std::string deviceName_;
    uint32_t driverVersion_ = 0;
    Path path_ = Path::GaussianPoint;
    Params params_;
    glm::vec3 objectCenter_{0.0f};
    struct ScanLevel { uint32_t base, size, parent; };
    std::vector<ScanLevel> scanLevels_;
    uint32_t scanWords_ = 1;
    uint32_t particleCapacity_ = 1;
    Camera camera_;
    const Phantom::PointCloud::GSPointCloud* cloud_ = nullptr;

    const Phantom::VKG::VulkanContext* ctx_ = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    uint32_t frames_ = kMaxFrames;
    uint32_t frameCounter_ = 0;
    uint32_t lastFrameIndex_ = 0;
    // Frames of update() elapsed since the last resetAccumulation() (Phase 3 Stats::framesSinceReset).
    uint32_t framesSinceReset_ = 0;

    // Progressive accumulation reset bookkeeping. resetPending_ counts down over
    // `frames_` frames so both frame-in-flight accumulators restart.
    uint32_t resetPending_ = kMaxFrames;
    glm::mat4 lastView_{ 0.0f };
    glm::vec3 lastCamPos_{ 0.0f }; // tracked apart from lastChangeHash_ (Phase 4 camera-only detection)
    uint64_t  lastChangeHash_ = 0;

    // Ensemble LOD bookkeeping (Phase 1). ensembleEpoch_ increments once per
    // resetAccumulation() call (i.e. once per history restart, shared by both
    // frame slots); ensembleHistory_[slot] is the cumulative count of ensembles
    // accumulated into that slot since its own last restart, reset to 0 the
    // moment that slot next observes resetAccum in update(). lastEffectiveR_
    // records what recordCompute() actually dispatched, for Stats reporting.
    uint32_t ensembleEpoch_ = 0;
    std::array<uint32_t, kMaxFrames> ensembleHistory_{};
    std::array<uint32_t, kMaxFrames> lastEffectiveR_{};

    // Adaptive ensemble LOD (Phase 2). lodController_.notifyMotion() is called
    // from resetAccumulation() -- the single choke point every history-invalidating
    // change (camera, params, data, resize, lodMode switch) already goes through --
    // so the controller sees every "Moving" trigger without recordCompute()/update()
    // needing to know which of those changes fired. adaptiveRequest_ is refreshed
    // once per frame in update() (where wall-clock dt and the last GPU timing are
    // available) and consumed by recordCompute() when Params.lodMode == Adaptive.
    EnsembleLodController lodController_;
    EnsembleLodController::Request adaptiveRequest_{1u, 1u};
    std::chrono::steady_clock::time_point lastUpdateTime_{};
    bool hasLastUpdateTime_ = false;

    // Pbvr3d particle-bank reuse (Phase 4). desiredBankEpoch_ increments only on
    // "hard" resetAccumulation() calls (data/params/resolution changes); a
    // camera-only change (Params.pbvrBankReuse path in update()) leaves it alone,
    // so bankBuiltEpoch_[slot] == desiredBankEpoch_ tells recordCompute() that
    // slot's bank (world position + Gaussian id, written by gps_compact.comp's
    // generate pass into bankBuf_[slot]) is still valid to reproject under the
    // new camera instead of resampling. Starts at 1 so the zero-initialized
    // bankBuiltEpoch_ never accidentally matches on the very first frame.
    uint32_t desiredBankEpoch_ = 1;
    std::array<uint32_t, kMaxFrames> bankBuiltEpoch_{};
    std::array<bool, kMaxFrames> lastBankReused_{};
    // A generate dispatch under compactPipeline==1 doesn't know at record time
    // whether the GPU will actually land in cached mode (gps_scan.comp decides
    // work[4] at *execution* time) -- only readable once that dispatch has
    // completed, one real frame later via this same slot's host-coherent
    // workBuf_. bankPendingEpoch_[slot] != 0 means "confirm on next visit to
    // this slot" (see update()); 0 means nothing pending.
    std::array<uint32_t, kMaxFrames> bankPendingEpoch_{};

    // ViewConditioned bank-drift invalidation (Phase 4 slice 2). The camera view
    // depth to objectCenter_ the bank currently in bankBuf_ was calibrated for --
    // overwritten on every generate dispatch (hard reset or drift-fallback
    // regeneration alike), so it always reflects the distance of whichever bank
    // content is actually resident. 0 means "no bank generated yet" (permissive).
    // Compared every reuse-eligible frame in recordCompute() when
    // Params.pbvr3dMethod == ViewConditioned; Proportional/Extinction ignore it.
    float bankRefViewDepth_ = 0.0f;

    VkExtent2D extent_{ 0, 0 };
    uint32_t   spp_ = 4;

    // input
    Phantom::VKG::VulkanBuffer gsInput_;
    Phantom::VKG::VulkanBuffer shRest_;      // SH rest coefficients (or 1 dummy float)
    uint32_t numSplats_ = 0;
    int      shDegreeData_ = 0;
    uint64_t cachedGeneration_ = ~0ull;

    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> preparedBuf_, scanBuf_, particleBuf_, workBuf_;
    // Pbvr3d particle bank (Phase 4): vec4(world.xyz, uintBitsToFloat(gaussianId))
    // per particle, same capacity/indexing as particleBuf_.
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> bankBuf_;
    Phantom::VKG::VulkanComputePipeline scanPipe_, compactPipe_;
    bool createWorkBuffers(const Phantom::VKG::VulkanContext&, const Phantom::VKG::VulkanCommandPool&);

    // per-frame buffers
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> depthBuf_;
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> colorBuf_;   // packed logarithmic HDR RGB per subpixel
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> accumBuf_;   // vec4 per pixel (rgb sum, count)
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> statsBuf_;    // host-visible
    std::array<Phantom::VKG::VulkanBuffer, kMaxFrames> paramsUbo_;   // host-visible
    Phantom::VKG::VulkanBuffer readbackBuf_;                         // validation-only

    // compute
    Phantom::VKG::VulkanDescriptorSetLayout computeDsl_;
    Phantom::VKG::VulkanComputePipeline     splatPipe_;    // gps_splat.comp   (GaussianPoint)
    Phantom::VKG::VulkanComputePipeline     pbvr3dPipe_;   // gps_pbvr3d.comp  (Pbvr3d)
    Phantom::VKG::VulkanComputePipeline     resolvePipe_;

    // composite (graphics)
    Phantom::VKG::VulkanDescriptorSetLayout compositeDsl_;
    Phantom::VKG::VulkanPipeline            compositePipe_;

    Phantom::VKG::VulkanDescriptorPool           descPool_;
    std::array<VkDescriptorSet, kMaxFrames>      computeSets_{};
    std::array<VkDescriptorSet, kMaxFrames>      compositeSets_{};

    // GPU timestamps: kMarks per frame (start, +clear, +depth, +color, +resolve,
    // then +prepare and +scan, which subdivide the depth interval; always all written).
    static constexpr uint32_t kMarks = 7;
    VkQueryPool queryPool_ = VK_NULL_HANDLE;
    uint32_t profileVariant_ = 0;
    float       tsPeriodNs_ = 0.0f;
    std::array<bool, kMaxFrames>  tsWritten_{};     // frame slot has been recorded at least once
    std::array<Stats, kMaxFrames> lastTimings_{};   // per-frame readback of the pass times
    double      budgetThin_ = 1.0;                  // adaptive stochastic-thinning scale

    void destroyFrameBuffers(VkDevice device);
    void writeComputeSet(VkDevice device, uint32_t f);
    void writeCompositeSet(VkDevice device, uint32_t f);
};

} // namespace GSView
