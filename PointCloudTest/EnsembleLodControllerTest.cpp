#include "pch.h"

// Phase 2 of docs/todo/PLAN_pbvr_gps_ensemble_lod.md: the pure CPU adaptive
// ensemble-count state machine behind GaussianPointRenderer's LodMode::Adaptive.
// No Vulkan, no wall clock -- dt/GPU time are injected so behaviour is deterministic.
#include "EnsembleLodController.h"

#include <algorithm>

using GSView::EnsembleLodController;
using State = EnsembleLodController::State;

namespace {

EnsembleLodController::Config testConfig()
{
    EnsembleLodController::Config cfg;
    cfg.settleMs = 100.0f;
    cfg.frameBudgetLowMs = 16.0f;
    cfg.frameBudgetHighMs = 33.0f;
    cfg.rMax = 8;
    cfg.targetMax = 1000; // large enough that convergence tests must set it explicitly
    cfg.emaAlpha = 1.0f;  // no smoothing -- makes the ramp deterministic per-call
    cfg.hysteresisMs = 0.0f;
    return cfg;
}

} // namespace

TEST(EnsembleLodController, StartsInMovingState)
{
    EnsembleLodController c(testConfig());
    EXPECT_EQ(c.state(), State::Moving);
}

TEST(EnsembleLodController, FirstCallTransitionsThroughMovingToSettling)
{
    EnsembleLodController c(testConfig());
    // No notifyMotion() call -- the controller starts in Moving with no motion
    // pending, so the very first advance() already begins settling.
    const auto req = c.advance(10.0f, 0.0f, true, 0);
    EXPECT_EQ(req.ensemblesPerFrame, 1u);
    EXPECT_EQ(c.state(), State::Settling);
}

TEST(EnsembleLodController, StaysAtR1UntilSettleDurationElapses)
{
    EnsembleLodController c(testConfig()); // settleMs = 100
    for (int i = 0; i < 9; ++i) {
        const auto req = c.advance(10.0f, 0.0f, true, 0); // 9 * 10ms = 90ms < 100ms
        EXPECT_EQ(req.ensemblesPerFrame, 1u);
        EXPECT_EQ(c.state(), State::Settling) << "iteration " << i;
    }
    // 10th call crosses the 100ms threshold.
    const auto req = c.advance(10.0f, 0.0f, true, 0);
    EXPECT_EQ(req.ensemblesPerFrame, 1u); // R only starts ramping on the *next* Refining call
    EXPECT_EQ(c.state(), State::Refining);
}

TEST(EnsembleLodController, RampsUpWithinBudgetThenStabilizes)
{
    EnsembleLodController c(testConfig()); // low budget 16ms, ~5ms per ensemble simulated below
    for (int i = 0; i < 10; ++i) c.advance(10.0f, 0.0f, true, 0); // reach Refining, R still 1
    ASSERT_EQ(c.state(), State::Refining);

    // Simulate a render whose GPU cost is ~5ms per ensemble: feed back
    // computeMs = 5 * R actually requested last call.
    uint32_t r = 1;
    uint32_t lastR = 0;
    for (int i = 0; i < 20; ++i) {
        const float simulatedComputeMs = 5.0f * static_cast<float>(r);
        const auto req = c.advance(10.0f, simulatedComputeMs, true, 0);
        r = req.ensemblesPerFrame;
        ASSERT_LE(r, 8u);
        if (i > 10) {
            // Should have stabilized well before 20 iterations.
            if (lastR != 0) EXPECT_EQ(r, lastR);
        }
        lastR = r;
    }
    // floor(16 / 5) == 3: three 5ms ensembles fit the 16ms low-budget window,
    // a fourth would not (20ms > 16ms).
    EXPECT_EQ(r, 3u);
    EXPECT_EQ(c.state(), State::Refining);
}

TEST(EnsembleLodController, RampsDownWhenComputeExceedsHighBudget)
{
    EnsembleLodController c(testConfig());
    for (int i = 0; i < 10; ++i) c.advance(10.0f, 0.0f, true, 0); // reach Refining
    ASSERT_EQ(c.state(), State::Refining);

    // Ramp up first with a cheap simulated cost.
    uint32_t r = 1;
    for (int i = 0; i < 10; ++i) {
        const auto req = c.advance(10.0f, 5.0f * static_cast<float>(std::max(r, 1u)), true, 0);
        r = req.ensemblesPerFrame;
    }
    ASSERT_GT(r, 1u);

    // Now simulate a sudden, flat cost spike (e.g. the scene got heavier) well
    // above the high-budget threshold -- R must step back down towards 1.
    for (int i = 0; i < 10 && r > 1; ++i) {
        const auto req = c.advance(10.0f, 50.0f, true, 0);
        r = req.ensemblesPerFrame;
    }
    EXPECT_EQ(r, 1u);
}

TEST(EnsembleLodController, NotifyMotionForcesR1AndMovingRegardlessOfPriorState)
{
    EnsembleLodController c(testConfig());
    for (int i = 0; i < 10; ++i) c.advance(10.0f, 0.0f, true, 0);
    for (int i = 0; i < 5; ++i) c.advance(10.0f, 5.0f, true, 0); // ramp up somewhat
    ASSERT_EQ(c.state(), State::Refining);

    c.notifyMotion();
    const auto req = c.advance(10.0f, 5.0f, true, 0);
    EXPECT_EQ(req.ensemblesPerFrame, 1u);
    EXPECT_EQ(c.state(), State::Moving);

    // The very next call (no further motion) starts settling again, same as a
    // fresh controller -- motion must not leave stale ramp-up state behind.
    const auto req2 = c.advance(10.0f, 0.0f, true, 0);
    EXPECT_EQ(req2.ensemblesPerFrame, 1u);
    EXPECT_EQ(c.state(), State::Settling);
}

TEST(EnsembleLodController, ConvergesAtTargetAndResumesWhenTargetRaised)
{
    auto config = testConfig();
    config.targetMax = 5;
    EnsembleLodController c(config);

    for (int i = 0; i < 10; ++i) c.advance(10.0f, 0.0f, true, 0); // reach Refining
    ASSERT_EQ(c.state(), State::Refining);

    // displayedEnsembles has not yet reached the target.
    auto req = c.advance(10.0f, 1.0f, true, 4);
    EXPECT_EQ(c.state(), State::Refining);
    EXPECT_EQ(req.targetEnsembles, 5u);

    // Now report the target as reached.
    req = c.advance(10.0f, 1.0f, true, 5);
    EXPECT_EQ(c.state(), State::Converged);

    // Raising the target (config changes live, read every call) resumes refining.
    config.targetMax = 8;
    c.setConfig(config);
    req = c.advance(10.0f, 1.0f, true, 5);
    EXPECT_EQ(c.state(), State::Refining);
    EXPECT_EQ(req.targetEnsembles, 8u);
}

TEST(EnsembleLodController, NoTimestampSupportFallsBackToFixedR1)
{
    auto config = testConfig();
    config.targetMax = 3;
    EnsembleLodController c(config);

    // Even with plenty of settle time and a "cheap" reported cost, without GPU
    // timestamps the controller must never request more than R=1.
    for (int i = 0; i < 5; ++i) {
        const auto req = c.advance(10.0f, 1.0f, /*timestampsSupported=*/false, i);
        EXPECT_EQ(req.ensemblesPerFrame, 1u) << "iteration " << i;
    }
    // Converges once displayedEnsembles reaches the (small) target, same as the
    // timestamped path.
    const auto req = c.advance(10.0f, 1.0f, false, 3);
    EXPECT_EQ(c.state(), State::Converged);
}

TEST(EnsembleLodController, ConfigRoundTrips)
{
    EnsembleLodController c;
    auto config = testConfig();
    config.rMax = 4;
    c.setConfig(config);
    EXPECT_EQ(c.config().rMax, 4u);
    EXPECT_FLOAT_EQ(c.config().settleMs, config.settleMs);
}
