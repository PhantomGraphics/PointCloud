#include "NormalEstimatorView.h"

#include "NormalEstimator.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void NormalEstimatorView::onImGui(World& world, int activeSceneId,
                                   const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::NormalEstimator estimator;
        for (const auto& p : positions) {
            estimator.add(p);
        }
        estimator.estimate(searchRadius_);

        if (orientToViewpoint_) {
            estimator.orientTowardsViewpoint(glm::vec3(viewpointX_, viewpointY_, viewpointZ_));
        }

        const auto normals = estimator.getNormals();

        auto* result = world.addScene("NormalResult");
        for (size_t i = 0; i < positions.size() && i < normals.size(); ++i) {
            const auto& n = normals[i];
            glm::vec3 col = {
                (n.x + 1.0f) * 0.5f,
                (n.y + 1.0f) * 0.5f,
                (n.z + 1.0f) * 0.5f
            };
            result->add(positions[i], col);
        }
        result->setNormals(normals);

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");

    ImGui::Checkbox("Orient towards viewpoint", &orientToViewpoint_);
    if (orientToViewpoint_) {
        ImGui::SliderFloat("Viewpoint X", &viewpointX_, -10.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Viewpoint Y", &viewpointY_, -10.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Viewpoint Z", &viewpointZ_, -10.0f, 10.0f, "%.3f");
    }

    runButton_.show();
}

} // namespace VPC
