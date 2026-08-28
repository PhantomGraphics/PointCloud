#include "CurvatureBasedFilterView.h"

#include "CurvatureEstimator.h"
#include "imgui.h"

#include <algorithm>
#include <glm/glm.hpp>

namespace VPC {

void CurvatureBasedFilterView::onImGui(World& world, int activeSceneId,
                                        const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr || curvatureThreshold_ < 0.0f) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::CurvatureEstimator estimator;
        for (const auto& p : positions) {
            estimator.add(p);
        }
        estimator.estimate(searchRadius_);

        const auto curvatures = estimator.getCurvatures();

        const double maxCurv = curvatures.empty()
            ? 1.0
            : *std::max_element(curvatures.begin(), curvatures.end());
        const double norm = (maxCurv > 0.0) ? maxCurv : 1.0;

        auto* result = world.addScene("CurvatureFilterResult");
        for (size_t i = 0; i < positions.size() && i < curvatures.size(); ++i) {
            if (curvatures[i] <= static_cast<double>(curvatureThreshold_)) {
                const float v = static_cast<float>(curvatures[i] / norm);
                result->add(positions[i], glm::vec3(v, v, v));
            }
        }

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Search Radius",        &searchRadius_,       0.001f, 1.0f, "%.4f");
    ImGui::SliderFloat("Curvature Threshold",  &curvatureThreshold_, 0.0f,   1.0f, "%.4f");
    runButton_.show();
}

} // namespace VPC
