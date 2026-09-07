#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class BoundaryDetectorView : public IProcessView {
public:
    const char* getName() const override { return "Boundary Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_      = 0.1f;
    float angleThresholdDeg_ = 153.0f; // ~0.85*pi, PCL BoundaryEstimation default

    bool                hasResult_ = false;
    ops::BoundaryResult result_;
};

} // namespace VPC
