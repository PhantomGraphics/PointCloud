#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

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

    bool        hasResult_   = false;
    bool        succeeded_   = false;
    float       nx_ = 0, ny_ = 0, nz_ = 0;
    float       offset_      = 0.0f;
    int         inlierCount_ = 0;
    std::string status_      = "Not executed yet";
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
