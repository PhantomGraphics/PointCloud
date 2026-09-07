#include "CurvatureEstimatorView.h"

#include "imgui.h"

namespace VPC {

void CurvatureEstimatorView::onImGui(World& world, int activeSceneId,
                                      const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::Checkbox("Principal Curvature (k1/k2)", &principalMode_);

    if (ImGui::Button("Run")) {
        ops::CurvatureParams p;
        p.radius    = searchRadius_;
        p.principal = principalMode_;
        result_    = ops::estimateCurvature(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    if (principalMode_ && hasResult_ && result_.outcome.ok) {
        ImGui::Text("Mean k1: %.5f", result_.meanK1);
        ImGui::Text("Mean k2: %.5f", result_.meanK2);
    }
    if (hasResult_ && !result_.outcome.ok)
        ImGui::TextColored(ImVec4(1.f, 0.5f, 0.3f, 1.f), "%s", result_.outcome.message.c_str());
}

} // namespace VPC
