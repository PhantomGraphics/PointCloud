#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class BoundaryDetectorView : public IProcessView {
public:
    const char* getName() const override { return "Boundary Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float searchRadius_      = 0.1f;
    float angleThresholdDeg_ = 153.0f; // ~0.85*pi, PCL BoundaryEstimation default

    bool        hasResult_     = false;
    bool        succeeded_     = false;
    std::string status_        = "Not executed yet";
    int         boundaryCount_ = 0;

    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
