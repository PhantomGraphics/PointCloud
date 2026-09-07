#include "DBSCANClusteringView.h"

#include "imgui.h"

namespace VPC {

void DBSCANClusteringView::onImGui(World& world, int activeSceneId,
                                    const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Eps",     &eps_,    0.001f, 1.0f, "%.4f");
    ImGui::SliderInt  ("Min Pts", &minPts_, 1,      100);

    if (ImGui::Button("Run")) {
        ops::DbscanParams p;
        p.eps    = eps_;
        p.minPts = minPts_;
        result_    = ops::clusterDbscan(world, activeSceneId, p);
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
