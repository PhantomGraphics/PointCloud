#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class GlobalRegistrationView : public IProcessView {
public:
    const char* getName() const override { return "Global Registration (FPFH+RANSAC)"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    int   targetId_       = -1;
    int   fpfhK_           = 20;
    int   iterations_      = 1000;
    float maxCorrDistance_ = 0.05f;
    int   sampleSize_      = 3;
    float edgeLengthTolerance_ = 0.15f;
    int   minInliers_      = 3;

    bool                     hasResult_ = false;
    ops::GlobalRegisterResult result_;
};

} // namespace VPC
