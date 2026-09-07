#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include "CGLib/UIWidgets/Vector3dView.h"
#include "CGLib/Math/Vector3d.h"

namespace VPC {

class PCRectView : public IProcessView {
public:
    const char* getName() const override { return "Generate Rect"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    Phantom::UI::Vector3dView originView_{ "Origin" };
    Phantom::UI::Vector3dView uvecView_{ "U Vec", Phantom::Math::Vector3df(1.0f, 0.0f, 0.0f) };
    Phantom::UI::Vector3dView vvecView_{ "V Vec", Phantom::Math::Vector3df(0.0f, 0.0f, 1.0f) };
    int   uCount_    = 50;
    int   vCount_    = 50;
    Phantom::UI::Button generateButton_{ "Generate" };
};

} // namespace VPC
