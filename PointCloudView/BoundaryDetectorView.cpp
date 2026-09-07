#include "BoundaryDetectorView.h"

#include "BoundaryDetector.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void BoundaryDetectorView::onImGui(World& world, int activeSceneId,
                                    const std::function<void(int)>& onResult)
{
    runButton_.setFunction([&world, activeSceneId, &onResult, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }
        if (!scene->hasNormals()) { status_ = "Scene has no normals - run Normal Estimator first"; return; }

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        Phantom::PC::BoundaryDetector detector;
        for (size_t i = 0; i < positions.size(); ++i) detector.add(positions[i], normals[i]);

        const float angleThresholdRad = angleThresholdDeg_ * 3.14159265f / 180.0f;
        detector.estimate(static_cast<double>(searchRadius_), angleThresholdRad);
        const auto flags = detector.getBoundaryFlags();

        auto* result = world.addScene("BoundaryResult");
        int count = 0;
        for (size_t i = 0; i < positions.size() && i < flags.size(); ++i) {
            if (flags[i]) {
                result->add(positions[i], glm::vec3(0.9f, 0.15f, 0.15f));
                ++count;
            } else {
                result->add(positions[i], glm::vec3(0.2f, 0.4f, 0.9f));
            }
        }

        scene->setVisible(false);
        onResult(-1);

        hasResult_     = true;
        succeeded_     = true;
        boundaryCount_ = count;
        status_        = "Boundary detection complete";
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderFloat("Angle Threshold (deg)", &angleThresholdDeg_, 10.0f, 179.0f, "%.1f");

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Boundary Points: %d", boundaryCount_);
    }
    runButton_.show();
}

} // namespace VPC
