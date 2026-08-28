#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class RansacSphereDetectorView : public IProcessView {
public:
    const char* getName() const override { return "RANSAC Sphere Detector"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float threshold_  = 0.02f;
    int   iterations_ = 200;
    int   minInliers_ = 50;

    bool        hasResult_   = false;
    bool        succeeded_   = false;
    float       cx_ = 0, cy_ = 0, cz_ = 0;
    float       radius_      = 0.0f;
    int         inlierCount_ = 0;
    std::string status_      = "Not executed yet";
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
