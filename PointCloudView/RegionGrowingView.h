#pragma once

#include "IProcessView.h"
#include "CGLib/UIWidgets/Button.h"
#include <string>

namespace VPC {

/// @brief UI for Phantom::PC::RegionGrowing (normal/curvature-based segmentation), distinct
/// from DistanceBasedClusteringView (menu label "Region Growing (Distance)").
class RegionGrowingView : public IProcessView {
public:
    const char* getName() const override { return "Region Growing (Normal/Curvature)"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void()>& onRebuild) override;

private:
    float curvatureRadius_       = 0.1f;
    int   kNeighbors_            = 30;
    float smoothnessThresholdDeg_ = 5.0f;
    float curvatureThreshold_    = 1.0f;
    int   minClusterSize_        = 10;

    bool        hasResult_    = false;
    bool        succeeded_    = false;
    std::string status_       = "Not executed yet";
    size_t      clusterCount_ = 0;
    Phantom::UI::Button runButton_{ "Run" };
};

} // namespace VPC
