#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class DensityEstimatorView : public IProcessView {
public:
    const char* getName() const override { return "Density Estimator"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_ = 0.05f;
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
