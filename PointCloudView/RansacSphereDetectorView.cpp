#include "RansacSphereDetectorView.h"

#include "RansacSphereDetector.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <vector>

namespace VPC {

void RansacSphereDetectorView::onImGui(World& world, int activeSceneId,
                                        const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();

        Phantom::PC::RansacSphereDetector detector;
        Phantom::PC::RansacSphereDetector::SphereModel model;

        const bool ok = detector.detect(
            positions, model, iterations_, threshold_,
            static_cast<size_t>(minInliers_));

        if (!ok) { status_ = "Detection failed"; return; }

        std::vector<bool> isInlier(positions.size(), false);

        auto* inliers = world.addScene("SphereInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.9f, 0.6f, 0.1f));
            }
        }

        auto* outliers = world.addScene("SphereOutliers");
        for (size_t i = 0; i < positions.size(); ++i) {
            if (!isInlier[i]) {
                outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
            }
        }

        scene->setVisible(false);
        onRebuild();

        hasResult_   = true;
        succeeded_   = true;
        cx_ = model.center.x; cy_ = model.center.y; cz_ = model.center.z;
        radius_      = model.radius;
        inlierCount_ = static_cast<int>(model.inliers.size());
        status_      = "Sphere detected";
    });

    ImGui::SliderFloat("Threshold",   &threshold_,  0.001f, 0.5f,  "%.4f");
    ImGui::SliderInt  ("Iterations",  &iterations_, 10,     1000);
    ImGui::SliderInt  ("Min Inliers", &minInliers_, 3,      5000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Center: (%.5f, %.5f, %.5f)", cx_, cy_, cz_);
        ImGui::Text("Radius: %.5f", radius_);
        ImGui::Text("Inliers: %d",  inlierCount_);
    }
    runButton_.show();
}

} // namespace VPC
