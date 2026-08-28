#include "DensityEstimatorView.h"

#include "DensityEstimator.h"
#include "imgui.h"

#include <algorithm>
#include <glm/glm.hpp>

namespace VPC {

void DensityEstimatorView::onImGui(World& world, int activeSceneId,
                                    const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::DensityEstimator estimator;
        for (const auto& p : positions) {
            estimator.add(p);
        }
        estimator.estimate(searchRadius_);

        const auto densities = estimator.getDensities();

        const double maxDensity = densities.empty()
            ? 1.0
            : *std::max_element(densities.begin(), densities.end());
        const double norm = (maxDensity > 0.0) ? maxDensity : 1.0;

        auto* result = world.addScene("DensityResult");
        for (size_t i = 0; i < positions.size(); ++i) {
            const float v = static_cast<float>(densities[i] / norm);
            result->add(positions[i], glm::vec3(v, v, v));
        }

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    runButton_.show();
}

} // namespace VPC
