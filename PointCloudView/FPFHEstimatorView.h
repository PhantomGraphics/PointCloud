#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

/// @brief UI for Phantom::PC::FPFHEstimator. The 33-dim descriptor itself can't be visualized
/// directly, so each point is colored by its Euclidean distance (in descriptor space) from the
/// scene's mean descriptor -- a rough "feature distinctiveness" map.
class FPFHEstimatorView : public IProcessView {
public:
    const char* getName() const override { return "FPFH Estimator"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    int kNeighbors_ = 20;

    bool           hasResult_ = false;
    ops::FpfhResult result_;
};

} // namespace VPC
