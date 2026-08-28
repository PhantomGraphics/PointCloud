#include "ICPRegistrationView.h"

#include "ICPRegistration.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <string>

namespace VPC {

void ICPRegistrationView::onImGui(World& world, int activeSceneId,
                                   const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* source = world.findById(activeSceneId);
        if (source == nullptr) { status_ = "Scene not found"; return; }
        auto* target = world.findById(targetId_);
        if (target == nullptr) { status_ = "Target scene not selected"; return; }

        const auto kernel = static_cast<Phantom::PC::ICPRegistration::RobustKernel>(robustKernel_);

        Phantom::PC::ICPRegistration icp;
        Phantom::PC::ICPRegistration::Result result;
        bool ok = false;
        if (pointToPlane_) {
            if (!target->hasNormals()) { status_ = "Target scene has no normals"; return; }
            ok = icp.alignPointToPlane(
                source->getPositions(), target->getPositions(), target->getNormals(),
                result, maxIterations_, tolerance_, maxCorrespondenceDistance_,
                kernel, robustKernelDelta_);
        } else {
            ok = icp.align(
                source->getPositions(), target->getPositions(), result,
                maxIterations_, tolerance_, maxCorrespondenceDistance_,
                kernel, robustKernelDelta_, estimateScale_);
        }
        if (!ok) { status_ = "ICP alignment failed"; return; }

        auto* aligned = world.addScene(pointToPlane_ ? "ICPAlignedP2Plane" : "ICPAligned");
        for (const auto& p : source->getPositions()) {
            aligned->add(Phantom::PC::ICPRegistration::transformPoint(result, p),
                         glm::vec3(0.3f, 1.0f, 0.5f));
        }
        source->setVisible(false);
        onRebuild();

        hasResult_  = true;
        succeeded_  = true;
        fitness_    = result.fitness;
        iterations_ = result.iterations;
        converged_  = result.converged;
        scale_      = result.scale;
        status_     = "Alignment complete";
    });

    ImGui::Text("Source: active scene (id=%d)", activeSceneId);

    const std::string targetLabel = (targetId_ >= 0) ? ("id " + std::to_string(targetId_)) : "(select target)";
    if (ImGui::BeginCombo("Target Scene", targetLabel.c_str())) {
        for (const auto& s : world.getScenes()) {
            if (s->getId() == activeSceneId) continue;
            const bool selected = (s->getId() == targetId_);
            const std::string label = s->getName() + " (id " + std::to_string(s->getId()) + ")";
            if (ImGui::Selectable(label.c_str(), selected)) targetId_ = s->getId();
        }
        ImGui::EndCombo();
    }

    ImGui::Checkbox("Point-to-Plane", &pointToPlane_);
    if (pointToPlane_) {
        auto* target = world.findById(targetId_);
        if (target != nullptr && !target->hasNormals()) {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
                "Target scene has no normals - run Normal Estimator on it first.");
        }
    } else {
        ImGui::Checkbox("Estimate Scale", &estimateScale_);
    }

    ImGui::SliderInt("Max Iterations", &maxIterations_, 1, 200);
    ImGui::SliderFloat("Tolerance", &tolerance_, 1.0e-8f, 1.0e-3f, "%.8f");
    ImGui::SliderFloat("Max Correspondence Dist", &maxCorrespondenceDistance_, 0.0f, 2.0f, "%.4f");

    const char* kernels[] = { "None", "Huber", "Tukey" };
    ImGui::Combo("Robust Kernel", &robustKernel_, kernels, 3);
    if (robustKernel_ != 0) ImGui::SliderFloat("Kernel Delta", &robustKernelDelta_, 0.001f, 5.0f, "%.4f");

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Fitness: %.6f", fitness_);
        ImGui::Text("Iterations: %d", iterations_);
        ImGui::Text("Converged: %s", converged_ ? "Yes" : "No");
        if (!pointToPlane_) ImGui::Text("Scale: %.5f", scale_);
    }
    runButton_.show();
}

} // namespace VPC
