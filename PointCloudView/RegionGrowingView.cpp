#include "RegionGrowingView.h"

#include "imgui.h"

namespace VPC {

void RegionGrowingView::onImGui(World& world, int activeSceneId,
                                 const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Curvature Radius", &curvatureRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderInt  ("k Neighbors", &kNeighbors_, 5, 100);
    ImGui::SliderFloat("Smoothness Threshold (deg)", &smoothnessThresholdDeg_, 0.5f, 45.0f, "%.2f");
    ImGui::SliderFloat("Curvature Threshold", &curvatureThreshold_, 0.001f, 5.0f, "%.4f");
    ImGui::SliderInt  ("Min Cluster Size", &minClusterSize_, 1, 1000);

    if (ImGui::Button("Run")) {
        ops::RegionGrowParams p;
        p.curvatureRadius    = curvatureRadius_;
        p.kNeighbors         = kNeighbors_;
        p.smoothnessDeg      = smoothnessThresholdDeg_;
        p.curvatureThreshold = curvatureThreshold_;
        p.minClusterSize     = minClusterSize_;
        result_    = ops::segmentRegionGrowing(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    ImGui::Separator();
    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
    } else {
        ImGui::TextDisabled("Not executed yet");
    }
}

} // namespace VPC
