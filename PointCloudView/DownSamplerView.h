#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class DownSamplerView : public IProcessView {
public:
    const char* getName() const override { return "Down Sampler"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float cellSize_ = 0.05f;
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
