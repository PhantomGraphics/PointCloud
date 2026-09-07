#include "ConvexHull2DView.h"

#include "imgui.h"

namespace VPC {

void ConvexHull2DView::onImGui(World& world, int activeSceneId,
                                const std::function<void(int)>& onResult)
{
    ImGui::TextWrapped("Computes the convex hull of the active scene's XY projection.");

    if (ImGui::Button("Run")) {
        result_    = ops::convexHull2D(world, activeSceneId);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    ImGui::Separator();
    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
        if (ok) {
            ImGui::Text("Vertices: %d", result_.vertexCount);
            ImGui::Text("Area: %.5f", result_.area);
        }
    } else {
        ImGui::TextDisabled("Not executed yet");
    }
}

} // namespace VPC
