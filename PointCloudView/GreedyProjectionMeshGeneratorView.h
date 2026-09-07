#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"

namespace VPC {

class GreedyProjectionMeshGeneratorView : public IProcessView {
public:
    const char* getName() const override { return "Greedy Projection Mesh"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    Phantom::UI::Button runButton_{ "Generate Mesh" };
    std::string status_;
    int triangleCount_ = 0;
};

} // namespace VPC
