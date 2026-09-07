#include "NormalEstimatorView.h"

#include "imgui.h"

namespace VPC {

void NormalEstimatorView::onImGui(World& world, int activeSceneId,
                                   const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");

    ImGui::Checkbox("Orient towards viewpoint", &orientToViewpoint_);
    if (orientToViewpoint_) {
        ImGui::SliderFloat("Viewpoint X", &viewpointX_, -10.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Viewpoint Y", &viewpointY_, -10.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Viewpoint Z", &viewpointZ_, -10.0f, 10.0f, "%.3f");
    }

    if (ImGui::Button("Run")) {
        ops::NormalParams p;
        p.radius            = searchRadius_;
        p.orientToViewpoint = orientToViewpoint_;
        p.viewpoint         = glm::vec3(viewpointX_, viewpointY_, viewpointZ_);
        lastOutcome_ = ops::estimateNormals(world, activeSceneId, p);
        if (lastOutcome_.ok) onResult(lastOutcome_.primarySceneId);
    }

    if (!lastOutcome_.message.empty()) {
        const ImVec4 col = lastOutcome_.ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                                           : ImVec4(1.0f, 0.5f, 0.3f, 1.0f);
        ImGui::TextColored(col, "%s", lastOutcome_.message.c_str());
    }
}

} // namespace VPC
