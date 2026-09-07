#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class ConvexHull2DView : public IProcessView {
public:
    const char* getName() const override { return "Convex Hull 2D"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    bool        hasResult_ = false;
    bool        succeeded_ = false;
    std::string status_    = "Not executed yet";
    float       area_        = 0.0f;
    int         vertexCount_ = 0;

    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
