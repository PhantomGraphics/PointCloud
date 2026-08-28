#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class CurvatureEstimatorView : public IProcessView {
public:
    const char* getName() const override { return "Curvature Estimator"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float searchRadius_ = 0.05f;
    bool  principalMode_ = false;

    double meanK1_ = 0.0, meanK2_ = 0.0;

    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
