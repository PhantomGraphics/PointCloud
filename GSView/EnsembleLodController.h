#pragma once

// -----------------------------------------------------------------------------
// EnsembleLodController -- CPU-only adaptive ensemble-count state machine
// (docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 2).
//
// GaussianPointRenderer's LodMode::Manual (Phase 1) lets the user fix
// ensemblesPerFrame/targetEnsembles by hand. LodMode::Adaptive instead asks
// this controller, once per displayed frame, how many independent ensembles R
// to dispatch and what cumulative target to stop at -- driven by GPU frame
// time and whether the scene just changed, per design principle 7 (decide
// next frame's iteration count from GPU time, no CPU wait).
//
// State machine: Moving -> Settling -> Refining -> Converged.
//   Moving    - a camera/param/data change was observed; R drops back to 1.
//   Settling  - no change since, but not yet quiet for `settleMs`; still R=1.
//   Refining  - quiet for `settleMs`; ramps R up/down within [1, rMax] to fill
//               the [frameBudgetLowMs, frameBudgetHighMs] window, using an EMA
//               of GPU compute time and a hysteresis band to avoid oscillating.
//   Converged - the displayed slot already holds `targetEnsembles` independent
//               samples; nothing left to generate. Raising the target (or any
//               scene change) leaves this state again.
//
// This class knows nothing about Vulkan, GaussianPointRenderer, or any other
// renderer detail -- it is a pure function of (motion signal, dt, last GPU
// time, displayed sample count) -> (R, target), so it can be unit-tested
// without a GPU (PointCloudTest EnsembleLodControllerTest.cpp) and reused by
// any LOD-driving render graph.
// -----------------------------------------------------------------------------

#include <algorithm>
#include <cstdint>

namespace GSView {

class EnsembleLodController {
public:
    // Initial candidates from docs/todo/PLAN_pbvr_gps_ensemble_lod.md Phase 2 sec.4
    // (Phase 3 is expected to expose these as UI/scenario-settable knobs and
    // confirm the numbers on real hardware -- this controller only needs them
    // as constructor/setConfig() input, it does not read them from anywhere).
    struct Config {
        float    settleMs          = 150.0f; // stillness duration before Settling -> Refining
        float    frameBudgetLowMs  = 16.7f;  // ramp R up while (ema + per-ensemble cost) stays under this
        float    frameBudgetHighMs = 33.3f;  // ramp R down once the ema GPU time exceeds this
        uint32_t rMax               = 8;      // hard cap on ensembles dispatched per displayed frame
        uint32_t targetMax          = 128;    // cumulative independent samples to converge at
        float    emaAlpha          = 0.25f;  // GPU compute-time moving-average weight (0..1)
        float    hysteresisMs      = 1.5f;   // margin required before stepping R up or down
    };

    enum class State { Moving = 0, Settling = 1, Refining = 2, Converged = 3 };

    struct Request { uint32_t ensemblesPerFrame; uint32_t targetEnsembles; };

    explicit EnsembleLodController(const Config& cfg = {}) : cfg_(cfg) {}

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // Call whenever the renderer restarts the sample history (camera/resolution/
    // data/probability-model change -- i.e. wherever GaussianPointRenderer
    // already calls resetAccumulation()). Takes effect on the next advance().
    void notifyMotion() { pendingMotion_ = true; }

    // Call once per displayed frame, after notifyMotion() for this frame (if
    // any). `lastComputeMs` is the most recent GPU compute-pass time available
    // for the displayed slot (0 if unknown); `timestampsSupported` false makes
    // this fall back to a fixed R=1 ramp (design principle: GPU timestamps
    // unavailable -> fixed R, plan sec.4 Phase 2). `displayedEnsembles` is the
    // slot's current cumulative independent-sample count.
    Request advance(float dtMs, float lastComputeMs, bool timestampsSupported,
                     uint32_t displayedEnsembles);

    State state() const { return state_; }

private:
    Config cfg_;
    State  state_ = State::Moving;
    bool   pendingMotion_ = false;
    float  stillMs_ = 0.0f;
    float  emaComputeMs_ = 0.0f;
    uint32_t currentR_ = 1;
};

} // namespace GSView
