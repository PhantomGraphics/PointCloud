#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

class GroundExtractorView : public IProcessView {
public:
    const char* getName() const override { return "Ground Extractor"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float cellSize_                  = 1.0f;
    float slope_                     = 0.3f;
    float initialWindowSize_         = 1.0f;
    float maxWindowSize_             = 16.0f;
    float windowGrowthFactor_        = 2.0f;
    float initialElevationThreshold_ = 0.2f;
    float maxElevationThreshold_     = 3.0f;
    float finalElevationThreshold_   = 0.3f;

    bool        hasResult_    = false;
    bool        succeeded_    = false;
    std::string status_       = "Not executed yet";
    int         groundCount_  = 0;
    int         nonGroundCount_ = 0;
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
