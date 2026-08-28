#include "RansacConeDetectorView.h"

#include "RansacConeDetector.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <vector>

namespace VPC {

void RansacConeDetectorView::onImGui(World& world, int activeSceneId,
                                      const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();

        Phantom::PC::RansacConeDetector detector;
        Phantom::PC::RansacConeDetector::ConeModel model;

        const bool ok = detector.detect(
            positions, model, iterations_, threshold_,
            static_cast<size_t>(minInliers_));

        if (!ok) { status_ = "Detection failed"; return; }

        std::vector<bool> isInlier(positions.size(), false);

        auto* inliers = world.addScene("ConeInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(1.0f, 0.7f, 0.2f));
            }
        }

        auto* outliers = world.addScene("ConeOutliers");
        for (size_t i = 0; i < positions.size(); ++i) {
            if (!isInlier[i]) {
                outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
            }
        }

        scene->setVisible(false);
        onRebuild();

        hasResult_    = true;
        succeeded_    = true;
        apexX_ = model.apex.x; apexY_ = model.apex.y; apexZ_ = model.apex.z;
        axisX_ = model.axis.x; axisY_ = model.axis.y; axisZ_ = model.axis.z;
        halfAngleRad_ = model.halfAngleRad;
        inlierCount_  = static_cast<int>(model.inliers.size());
        status_       = "Cone detected";
    });

    ImGui::SliderFloat("Threshold",   &threshold_,  0.001f, 0.5f,  "%.4f");
    ImGui::SliderInt  ("Iterations",  &iterations_, 10,     1000);
    ImGui::SliderInt  ("Min Inliers", &minInliers_, 3,      5000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Apex: (%.5f, %.5f, %.5f)", apexX_, apexY_, apexZ_);
        ImGui::Text("Axis: (%.5f, %.5f, %.5f)", axisX_, axisY_, axisZ_);
        ImGui::Text("Half Angle: %.5f rad (%.2f deg)", halfAngleRad_, halfAngleRad_ * 180.0f / 3.14159265f);
        ImGui::Text("Inliers: %d", inlierCount_);
    }
    runButton_.show();
}

} // namespace VPC
