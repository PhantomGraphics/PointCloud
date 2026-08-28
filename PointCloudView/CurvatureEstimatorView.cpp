#include "CurvatureEstimatorView.h"

#include "CurvatureEstimator.h"
#include "imgui.h"

#include <algorithm>
#include <glm/glm.hpp>

namespace VPC {

void CurvatureEstimatorView::onImGui(World& world, int activeSceneId,
                                      const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::CurvatureEstimator estimator;
        for (const auto& p : positions) {
            estimator.add(p);
        }

        if (principalMode_) {
            estimator.estimatePrincipal(searchRadius_);
            const auto pc = estimator.getPrincipalCurvatures();

            double maxAbs = 0.0, sumK1 = 0.0, sumK2 = 0.0;
            for (const auto& c : pc) {
                maxAbs = std::max({ maxAbs, std::abs(c.k1), std::abs(c.k2) });
                sumK1 += c.k1; sumK2 += c.k2;
            }
            const double norm = (maxAbs > 0.0) ? maxAbs : 1.0;

            auto* result = world.addScene("PrincipalCurvatureResult");
            for (size_t i = 0; i < positions.size() && i < pc.size(); ++i) {
                const float v = static_cast<float>(std::max(std::abs(pc[i].k1), std::abs(pc[i].k2)) / norm);
                result->add(positions[i], glm::vec3(v, v, v));
            }
            meanK1_ = pc.empty() ? 0.0 : sumK1 / static_cast<double>(pc.size());
            meanK2_ = pc.empty() ? 0.0 : sumK2 / static_cast<double>(pc.size());
        } else {
            estimator.estimate(searchRadius_);
            const auto curvatures = estimator.getCurvatures();

            const double maxCurv = curvatures.empty()
                ? 1.0
                : *std::max_element(curvatures.begin(), curvatures.end());
            const double norm = (maxCurv > 0.0) ? maxCurv : 1.0;

            auto* result = world.addScene("CurvatureResult");
            for (size_t i = 0; i < positions.size() && i < curvatures.size(); ++i) {
                const float v = static_cast<float>(curvatures[i] / norm);
                result->add(positions[i], glm::vec3(v, v, v));
            }
        }

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::Checkbox("Principal Curvature (k1/k2)", &principalMode_);
    if (principalMode_) {
        ImGui::Text("Mean k1: %.5f", meanK1_);
        ImGui::Text("Mean k2: %.5f", meanK2_);
    }
    runButton_.show();
}

} // namespace VPC
