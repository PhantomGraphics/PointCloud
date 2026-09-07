#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class CurvatureEstimatorView : public IProcessView {
public:
    const char* getName() const override { return "Curvature Estimator"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_  = 0.05f;
    bool  principalMode_ = false;

    bool                 hasResult_ = false;
    ops::CurvatureResult result_;
};

} // namespace VPC
