#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class CurvatureBasedFilterView : public IProcessView {
public:
    const char* getName() const override { return "Curvature Filter"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float curvatureThreshold_ = 0.05f;
    float searchRadius_       = 0.01f;
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
