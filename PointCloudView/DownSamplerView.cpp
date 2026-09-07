#include "DownSamplerView.h"

#include "imgui.h"

namespace VPC {

void DownSamplerView::onImGui(World& world, int activeSceneId,
                              const std::function<void(int)>& onResult)
{
    ImGui::SliderFloat("Cell Size", &cellSize_, 0.001f, 1.0f, "%.4f");

    if (ImGui::Button("Run")) {
        ops::DownSampleParams p;
        p.cellSize = cellSize_;
        // Same typed function the scenario CommandDispatcher calls, so the GUI
        // and DownSample: command produce identical points / colour / visibility
        // / selection (PLAN Phase 4).
        lastOutcome_ = ops::downSample(world, activeSceneId, p);
        if (lastOutcome_.ok)
            onResult(lastOutcome_.resultSceneId);  // select the result + rebuild
    }

    if (!lastOutcome_.message.empty()) {
        const ImVec4 col = lastOutcome_.ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                                           : ImVec4(1.0f, 0.5f, 0.3f, 1.0f);
        ImGui::TextColored(col, "%s", lastOutcome_.message.c_str());
    }
}

} // namespace VPC
