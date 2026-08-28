#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class RansacCylinderDetectorView : public IProcessView {
public:
    const char* getName() const override { return "RANSAC Cylinder Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float threshold_  = 0.01f;
    int   iterations_ = 100;
    int   minInliers_ = 50;

    bool        hasResult_   = false;
    bool        succeeded_   = false;
    float       ax_ = 0, ay_ = 0, az_ = 0;
    float       radius_      = 0.0f;
    int         inlierCount_ = 0;
    std::string status_      = "Not executed yet";
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
