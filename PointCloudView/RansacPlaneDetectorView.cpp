#include "RansacPlaneDetectorView.h"

#include "imgui.h"

namespace VPC {

void RansacPlaneDetectorView::onImGui(World& world, int activeSceneId,
                                       const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Threshold",   &threshold_,  0.001f, 0.5f, "%.4f");
    ImGui::SliderInt  ("Iterations",  &iterations_, 10,     1000);
    ImGui::SliderInt  ("Min Inliers", &minInliers_, 3,      5000);

    if (ImGui::Button("Run")) {
        ops::RansacParams p;
        p.threshold        = threshold_;
        p.iterations       = iterations_;
        p.minInliers       = minInliers_;
        p.buildOverlayMesh = true;
        result_    = ops::detectPlane(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    ImGui::Separator();
    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
        if (ok) {
            ImGui::Text("Normal: (%.5f, %.5f, %.5f)", result_.normal.x, result_.normal.y, result_.normal.z);
            ImGui::Text("Offset: %.5f", result_.offset);
            ImGui::Text("Inliers: %d", result_.inlierCount);
        }
    } else {
        ImGui::TextDisabled("Not executed yet");
    }
}

} // namespace VPC
