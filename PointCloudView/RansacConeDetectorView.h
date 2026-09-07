#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class RansacConeDetectorView : public IProcessView {
public:
    const char* getName() const override { return "RANSAC Cone Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float threshold_  = 0.02f;
    int   iterations_ = 200;
    int   minInliers_ = 50;

    bool        hasResult_    = false;
    bool        succeeded_    = false;
    float       apexX_ = 0, apexY_ = 0, apexZ_ = 0;
    float       axisX_ = 0, axisY_ = 0, axisZ_ = 0;
    float       halfAngleRad_ = 0.0f;
    int         inlierCount_  = 0;
    std::string status_       = "Not executed yet";
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
