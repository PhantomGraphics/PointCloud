#include "DistanceBasedClusteringView.h"

#include "imgui.h"

namespace VPC {

void DistanceBasedClusteringView::onImGui(World& world, int activeSceneId,
                                           const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");

    if (ImGui::Button("Run")) {
        ops::DistanceClusterParams p;
        p.radius = searchRadius_;
        result_    = ops::clusterDistance(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
    }
}

} // namespace VPC
