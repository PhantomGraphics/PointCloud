#include "ConcaveHull2DView.h"

#include "imgui.h"

namespace VPC {

void ConcaveHull2DView::onImGui(World& world, int activeSceneId,
                                 const std::function<void(int)>& onResult)
{
    ImGui::TextWrapped("Computes the concave hull of the active scene's XY projection.");
    ImGui::SliderInt("Initial k", &k_, 3, 30);
    ImGui::SliderInt("Max k (0 = unbounded)", &maxK_, 0, 200);

    if (ImGui::Button("Run")) {
        ops::ConcaveHullParams p;
        p.k    = k_;
        p.maxK = maxK_;
        result_    = ops::concaveHull2D(world, activeSceneId, p);
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
