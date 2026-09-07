#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class RansacSphereDetectorView : public IProcessView {
public:
    const char* getName() const override { return "RANSAC Sphere Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float threshold_  = 0.02f;
    int   iterations_ = 200;
    int   minInliers_ = 50;

    bool                   hasResult_ = false;
    ops::RansacSphereResult result_;
};

} // namespace VPC
