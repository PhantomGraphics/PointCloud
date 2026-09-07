#include "GroundExtractorView.h"

#include "imgui.h"

namespace VPC {

void GroundExtractorView::onImGui(World& world, int activeSceneId,
                                   const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Cell Size", &params_.cellSize, 0.05f, 10.0f, "%.3f");
    ImGui::SliderFloat("Slope",     &params_.slope,    0.01f, 2.0f,  "%.3f");

    if (ImGui::CollapsingHeader("Advanced")) {
        ImGui::SliderFloat("Initial Window Size", &params_.initialWindowSize, 0.1f, 10.0f, "%.3f");
        ImGui::SliderFloat("Max Window Size", &params_.maxWindowSize, 1.0f, 64.0f, "%.2f");
        ImGui::SliderFloat("Window Growth Factor", &params_.windowGrowthFactor, 1.01f, 4.0f, "%.3f");
        ImGui::SliderFloat("Initial Elevation Threshold", &params_.initialElevationThreshold, 0.01f, 5.0f, "%.3f");
        ImGui::SliderFloat("Max Elevation Threshold", &params_.maxElevationThreshold, 0.1f, 20.0f, "%.3f");
        ImGui::SliderFloat("Final Elevation Threshold", &params_.finalElevationThreshold, 0.01f, 5.0f, "%.3f");
    }

    if (ImGui::Button("Run")) {
        result_    = ops::extractGround(world, activeSceneId, params_);
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
