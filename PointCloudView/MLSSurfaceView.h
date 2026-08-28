#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class MLSSurfaceView : public IProcessView {
public:
    const char* getName() const override { return "MLS Surface"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float searchRadius_   = 0.1f;
    float upsampleRadius_ = 0.05f;
    float stepSize_       = 0.01f;

    std::string status_ = "Not executed yet";
    Phantom::UI::Button smoothButton_{ "Smooth" };
    Phantom::UI::Button upsampleButton_{ "Upsample" };
};

} // namespace VPC
