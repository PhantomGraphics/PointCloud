#include "GlobalRegistrationView.h"

#include "FPFHEstimator.h"
#include "GlobalRegistration.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <string>

namespace VPC {

void GlobalRegistrationView::onImGui(World& world, int activeSceneId,
                                      const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* source = world.findById(activeSceneId);
        if (source == nullptr) { status_ = "Scene not found"; return; }
        auto* target = world.findById(targetId_);
        if (target == nullptr) { status_ = "Target scene not selected"; return; }
        if (!source->hasNormals() || !target->hasNormals()) {
            status_ = "Source and target must both have normals";
            return;
        }

        Phantom::PC::FPFHEstimator sourceFpfh, targetFpfh;
        const auto& sp = source->getPositions();
        const auto& sn = source->getNormals();
        for (size_t i = 0; i < sp.size(); ++i) sourceFpfh.add(sp[i], sn[i]);
        const auto& tp = target->getPositions();
        const auto& tn = target->getNormals();
        for (size_t i = 0; i < tp.size(); ++i) targetFpfh.add(tp[i], tn[i]);

        if (!sourceFpfh.estimate(static_cast<size_t>(fpfhK_)) ||
            !targetFpfh.estimate(static_cast<size_t>(fpfhK_))) {
            status_ = "FPFH estimation failed";
            return;
        }

        Phantom::PC::GlobalRegistration globalReg;
        Phantom::PC::GlobalRegistration::Result result;
        const bool ok = globalReg.align(
            sp, sourceFpfh.getHistograms(), tp, targetFpfh.getHistograms(), result,
            iterations_, maxCorrDistance_, static_cast<size_t>(sampleSize_),
            edgeLengthTolerance_, static_cast<size_t>(minInliers_));
        if (!ok) { status_ = "Global registration failed"; return; }

        auto* aligned = world.addScene("GlobalRegAligned");
        for (const auto& p : sp) {
            aligned->add(result.rotation * p + result.translation, glm::vec3(1.0f, 0.5f, 0.8f));
        }
        source->setVisible(false);
        onRebuild();

        hasResult_   = true;
        succeeded_   = true;
        inlierCount_ = result.inlierCount;
        inlierRmse_  = result.inlierRmse;
        status_      = "Registration complete";
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

    auto* source = world.findById(activeSceneId);
    auto* target = world.findById(targetId_);
    if ((source != nullptr && !source->hasNormals()) || (target != nullptr && !target->hasNormals())) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
            "Source and target both need normals - run Normal Estimator on each first.");
    }

    ImGui::SliderInt("FPFH k Neighbors", &fpfhK_, 5, 100);
    ImGui::SliderInt("RANSAC Iterations", &iterations_, 100, 5000);
    ImGui::SliderFloat("Max Correspondence Dist", &maxCorrDistance_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderInt("Sample Size", &sampleSize_, 3, 6);
    ImGui::SliderFloat("Edge Length Tolerance", &edgeLengthTolerance_, 0.01f, 1.0f, "%.3f");
    ImGui::SliderInt("Min Inliers", &minInliers_, 3, 1000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Inlier Count: %zu", inlierCount_);
        ImGui::Text("Inlier RMSE: %.6f", inlierRmse_);
    }
    runButton_.show();
}

} // namespace VPC
