#include "DensityBasedFilterView.h"

#include "imgui.h"

namespace VPC {

void DensityBasedFilterView::onImGui(World& world, int activeSceneId,
                                      const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");

    if (ImGui::Button("Run")) {
        ops::DensityFilterParams p;
        p.radius = searchRadius_;
        lastOutcome_ = ops::filterDensity(world, activeSceneId, p);
        if (lastOutcome_.ok) onResult(lastOutcome_.primarySceneId);
    }

    if (!lastOutcome_.message.empty()) {
        const ImVec4 col = lastOutcome_.ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                                           : ImVec4(1.0f, 0.5f, 0.3f, 1.0f);
        ImGui::TextColored(col, "%s", lastOutcome_.message.c_str());
    }
}

} // namespace VPC
