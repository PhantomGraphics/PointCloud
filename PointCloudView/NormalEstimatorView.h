#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class NormalEstimatorView : public IProcessView {
public:
    const char* getName() const override { return "Normal Estimator"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_ = 0.05f;

    bool  orientToViewpoint_ = false;
    float viewpointX_ = 0.0f, viewpointY_ = 0.0f, viewpointZ_ = 0.0f;

    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
