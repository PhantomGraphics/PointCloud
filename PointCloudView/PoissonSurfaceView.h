#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class PoissonSurfaceView : public IProcessView {
public:
    const char* getName() const override { return "Poisson Surface"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    int   resolution_    = 64;
    float samplesPerCell_ = 1.5f;
    float isoLevel_      = 0.0f;
    int   maxIters_      = 200;
    float omega_         = 1.8f;
    float bboxPadding_   = 2.5f;
    bool  clampToBBox_   = true;

    Phantom::UI::Button runButton_{ "Reconstruct" };
    std::string status_;
    int vertexCount_   = 0;
    int triangleCount_ = 0;
};

} // namespace VPC
