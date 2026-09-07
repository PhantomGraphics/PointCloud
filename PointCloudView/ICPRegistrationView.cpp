#include "ICPRegistrationView.h"

#include "imgui.h"

#include <string>

namespace VPC {

void ICPRegistrationView::onImGui(World& world, int activeSceneId,
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

    ImGui::Checkbox("Point-to-Plane", &pointToPlane_);
    if (pointToPlane_) {
        auto* target = world.findById(targetId_);
        if (target != nullptr && !target->hasNormals())
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
                "Target scene has no normals - run Normal Estimator on it first.");
    } else {
        ImGui::Checkbox("Estimate Scale", &estimateScale_);
    }

    ImGui::SliderInt("Max Iterations", &maxIterations_, 1, 200);
    ImGui::SliderFloat("Tolerance", &tolerance_, 1.0e-8f, 1.0e-3f, "%.8f");
    ImGui::SliderFloat("Max Correspondence Dist", &maxCorrespondenceDistance_, 0.0f, 2.0f, "%.4f");

    const char* kernels[] = { "None", "Huber", "Tukey" };
    ImGui::Combo("Robust Kernel", &robustKernel_, kernels, 3);
    if (robustKernel_ != 0)
        ImGui::SliderFloat("Kernel Delta", &robustKernelDelta_, 0.001f, 5.0f, "%.4f");

    if (ImGui::Button("Run")) {
        ops::IcpParams p;
        p.targetSceneId             = targetId_;
        p.pointToPlane              = pointToPlane_;
        p.maxIterations             = maxIterations_;
        p.tolerance                 = tolerance_;
        p.maxCorrespondenceDistance = maxCorrespondenceDistance_;
        p.robustKernel              = robustKernel_;
        p.robustKernelDelta         = robustKernelDelta_;
        p.estimateScale             = estimateScale_;
        result_    = ops::icpAlign(world, activeSceneId, p);
        hasResult_ = true;
        if (result_.outcome.ok) onResult(result_.outcome.primarySceneId);
    }

    ImGui::Separator();
    if (hasResult_) {
        const bool ok = result_.outcome.ok;
        ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.5f, 0.3f, 1.f),
                           "%s", result_.outcome.message.c_str());
        if (ok) {
            ImGui::Text("Fitness: %.6f", result_.fitness);
            ImGui::Text("Iterations: %d", result_.iterations);
            ImGui::Text("Converged: %s", result_.converged ? "Yes" : "No");
            if (!pointToPlane_) ImGui::Text("Scale: %.5f", result_.scale);
        }
    } else {
        ImGui::TextDisabled("Not executed yet");
    }
}

} // namespace VPC
