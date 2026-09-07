#include "FPFHEstimatorView.h"

#include "imgui.h"

namespace VPC {

void FPFHEstimatorView::onImGui(World& world, int activeSceneId,
                                 const std::function<void(int)>& onResult)
{
    ImGui::SliderInt("k Neighbors", &kNeighbors_, 5, 100);

    if (ImGui::Button("Run")) {
        ops::FpfhParams p;
        p.kNeighbors = kNeighbors_;
        result_    = ops::estimateFpfh(world, activeSceneId, p);
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
