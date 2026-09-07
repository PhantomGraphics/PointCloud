#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class RansacPlaneDetectorView : public IProcessView {
public:
    const char* getName() const override { return "RANSAC Plane Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float threshold_  = 0.01f;
    int   iterations_ = 100;
    int   minInliers_ = 50;

    bool                  hasResult_ = false;
    ops::RansacPlaneResult result_;
};

} // namespace VPC
