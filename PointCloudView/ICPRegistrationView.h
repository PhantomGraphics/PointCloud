#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class ICPRegistrationView : public IProcessView {
public:
    const char* getName() const override { return "ICP Registration"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    int   targetId_          = -1;
    bool  pointToPlane_      = false;
    int   maxIterations_     = 50;
    float tolerance_         = 1.0e-6f;
    float maxCorrespondenceDistance_ = 0.0f;
    int   robustKernel_      = 0; // 0=None, 1=Huber, 2=Tukey
    float robustKernelDelta_ = 1.0f;
    bool  estimateScale_     = false;

    bool          hasResult_ = false;
    ops::IcpResult result_;
};

} // namespace VPC
