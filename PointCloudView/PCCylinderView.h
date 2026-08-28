#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/Vector3dView.h"

namespace VPC {

class PCCylinderView : public IProcessView {
public:
    const char* getName() const override { return "Generate Cylinder"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    Phantom::UI::Vector3dView centerView_{ "Center" };
    float radius_    = 0.5f;
    float height_    = 1.0f;
    int   count_     = 10000;
    Phantom::UI::Button generateButton_{ "Generate" };
};

} // namespace VPC
