#include "BoundaryDetectorView.h"

#include "imgui.h"

namespace VPC {

void BoundaryDetectorView::onImGui(World& world, int activeSceneId,
                                    const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderFloat("Angle Threshold (deg)", &angleThresholdDeg_, 10.0f, 179.0f, "%.1f");

    if (ImGui::Button("Run")) {
        ops::BoundaryParams p;
        p.radius            = searchRadius_;
        p.angleThresholdDeg = angleThresholdDeg_;
        result_    = ops::detectBoundary(world, activeSceneId, p);
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
