#include "EnsembleLodController.h"

#include <algorithm>

namespace GSView {

EnsembleLodController::Request EnsembleLodController::advance(
    float dtMs, float lastComputeMs, bool timestampsSupported, uint32_t displayedEnsembles)
{
    if (pendingMotion_) {
        pendingMotion_ = false;
        state_ = State::Moving;
        stillMs_ = 0.0f;
        currentR_ = 1;
    } else if (state_ == State::Moving) {
        // No motion was reported since the previous call -- start measuring
        // stillness from here (the frame that saw the change already ran with
        // R=1 above).
        state_ = State::Settling;
        stillMs_ = 0.0f;
    }

    if (!timestampsSupported) {
        // No GPU timing available: fall back to a fixed R=1 ramp rather than
        // guessing a budget (plan sec.4 Phase 2, design principle 7).
        currentR_ = 1;
        state_ = (displayedEnsembles >= cfg_.targetMax) ? State::Converged : State::Refining;
        return { currentR_, cfg_.targetMax };
    }

    if (state_ == State::Settling) {
        stillMs_ += dtMs;
        currentR_ = 1;
        if (stillMs_ >= cfg_.settleMs) {
            state_ = State::Refining;
            stillMs_ = 0.0f;
        }
    } else if (state_ == State::Refining) {
        if (lastComputeMs > 0.0f) {
            emaComputeMs_ = (emaComputeMs_ <= 0.0f)
                ? lastComputeMs
                : (cfg_.emaAlpha * lastComputeMs + (1.0f - cfg_.emaAlpha) * emaComputeMs_);
        }
        const float perEnsembleMs = (currentR_ > 0 && emaComputeMs_ > 0.0f)
            ? emaComputeMs_ / static_cast<float>(currentR_) : 0.0f;
        if (perEnsembleMs > 0.0f) {
            if (currentR_ < cfg_.rMax &&
                emaComputeMs_ + perEnsembleMs <= cfg_.frameBudgetLowMs - cfg_.hysteresisMs) {
                ++currentR_;
            } else if (currentR_ > 1 && emaComputeMs_ > cfg_.frameBudgetHighMs + cfg_.hysteresisMs) {
                --currentR_;
            }
        }
        if (displayedEnsembles >= cfg_.targetMax) state_ = State::Converged;
    } else if (state_ == State::Converged) {
        // A raised target (or a target lowered below the current count, which
        // this comparison also treats as "not converged") resumes refining.
        if (displayedEnsembles < cfg_.targetMax) state_ = State::Refining;
    }

    return { std::max(1u, currentR_), cfg_.targetMax };
}

} // namespace GSView
