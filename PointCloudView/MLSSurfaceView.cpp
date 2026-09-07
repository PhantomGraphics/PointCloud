#include "MLSSurfaceView.h"

#include "imgui.h"

namespace VPC {

void MLSSurfaceView::onImGui(World& world, int activeSceneId,
                              const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius",   &searchRadius_,   0.001f, 1.0f, "%.4f");
    ImGui::SliderFloat("Upsample Radius", &upsampleRadius_, 0.001f, 0.5f, "%.4f");
    ImGui::SliderFloat("Step Size",       &stepSize_,       0.001f, 0.2f, "%.4f");

    if (ImGui::Button("Smooth")) {
        ops::MlsSmoothParams p;
        p.radius = searchRadius_;
        lastOutcome_ = ops::mlsSmooth(world, activeSceneId, p);
        hasResult_ = true;
        if (lastOutcome_.ok) onResult(lastOutcome_.primarySceneId);
    }
    ImGui::SameLine();
    if (ImGui::Button("Upsample")) {
        ops::MlsUpsampleParams p;
        p.radius         = searchRadius_;
        p.upsampleRadius = upsampleRadius_;
        p.stepSize       = stepSize_;
        lastOutcome_ = ops::mlsUpsample(world, activeSceneId, p);
        hasResult_ = true;
        if (lastOutcome_.ok) onResult(lastOutcome_.primarySceneId);
    }

    if (hasResult_) {
        const bool ok = lastOutcome_.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", lastOutcome_.message.c_str());
    }
}

} // namespace VPC
