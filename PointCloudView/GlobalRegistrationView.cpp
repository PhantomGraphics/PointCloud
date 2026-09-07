#include "GlobalRegistrationView.h"

#include "imgui.h"

#include <string>

namespace VPC {

void GlobalRegistrationView::onImGui(World& world, int activeSceneId,
                                      const std::function<void(int)>& onResult)
{
    ImGui::Text("Source: active scene (id=%d)", activeSceneId);

    const std::string targetLabel = (targetId_ >= 0)
        ? ("id " + std::to_string(targetId_)) : "(select target)";
    if (ImGui::BeginCombo("Target Scene", targetLabel.c_str())) {
        for (const auto& s : world.getScenes()) {
            if (s->getId() == activeSceneId) continue;
            const std::string label = s->getName() + " (id " + std::to_string(s->getId()) + ")";
            if (ImGui::Selectable(label.c_str(), s->getId() == targetId_)) targetId_ = s->getId();
        }
        ImGui::EndCombo();
    }

    auto* source = world.findById(activeSceneId);
    auto* target = world.findById(targetId_);
    if ((source != nullptr && !source->hasNormals()) ||
        (target != nullptr && !target->hasNormals()))
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
            "Source and target both need normals - run Normal Estimator on each first.");

    ImGui::SliderInt("FPFH k Neighbors", &fpfhK_, 5, 100);
    ImGui::SliderInt("RANSAC Iterations", &iterations_, 100, 5000);
    ImGui::SliderFloat("Max Correspondence Dist", &maxCorrDistance_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderInt("Sample Size", &sampleSize_, 3, 6);
    ImGui::SliderFloat("Edge Length Tolerance", &edgeLengthTolerance_, 0.01f, 1.0f, "%.3f");
    ImGui::SliderInt("Min Inliers", &minInliers_, 3, 1000);

    if (ImGui::Button("Run")) {
        ops::GlobalRegisterParams p;
        p.targetSceneId              = targetId_;
        p.fpfhK                      = fpfhK_;
        p.iterations                 = iterations_;
        p.maxCorrespondenceDistance  = maxCorrDistance_;
        p.sampleSize                 = sampleSize_;
        p.edgeLengthTolerance        = edgeLengthTolerance_;
        p.minInliers                 = minInliers_;
        result_    = ops::globalRegister(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    ImGui::Separator();
    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
        if (ok) {
            ImGui::Text("Inlier Count: %d", result_.inlierCount);
            ImGui::Text("Inlier RMSE: %.6f", result_.inlierRmse);
        }
    } else {
        ImGui::TextDisabled("Not executed yet");
    }
}

} // namespace VPC
