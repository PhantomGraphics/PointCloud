#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class ICPRegistrationView : public IProcessView {
public:
    const char* getName() const override { return "ICP Registration"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    int   targetId_          = -1;
    bool  pointToPlane_      = false;
    int   maxIterations_     = 50;
    float tolerance_         = 1.0e-6f;
    float maxCorrespondenceDistance_ = 0.0f;
    int   robustKernel_      = 0; // 0=None, 1=Huber, 2=Tukey
    float robustKernelDelta_ = 1.0f;
    bool  estimateScale_     = false;

    bool        hasResult_  = false;
    bool        succeeded_  = false;
    std::string status_     = "Not executed yet";
    float       fitness_    = 0.0f;
    int         iterations_ = 0;
    bool        converged_  = false;
    float       scale_      = 1.0f;

    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
