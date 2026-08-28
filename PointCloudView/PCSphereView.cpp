#include "PCSphereView.h"

#include "imgui.h"

#include <cmath>
#include <glm/glm.hpp>
#include <random>

namespace VPC {

void PCSphereView::onImGui(World& world, int /*activeSceneId*/,
                            const std::function<void()>& onRebuild)
{
    generateButton_.setFunction([&world, &onRebuild, this]() {
        if (radius_ <= 0.0f || count_ <= 0) return;

        std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);

        const auto center = centerView_.getValue();
        const glm::vec3 cx{ center.x, center.y, center.z };

        auto* result = world.addScene("Sphere");
        for (int i = 0; i < count_; ++i) {
            const float u     = dist(rng);
            const float v     = dist(rng);
            const float theta = 2.0f * 3.14159265f * u;
            const float phi   = std::acosf(1.0f - 2.0f * v);
            glm::vec3 pos = cx + radius_ * glm::vec3{
                std::sinf(phi) * std::cosf(theta),
                std::sinf(phi) * std::sinf(theta),
                std::cosf(phi)
            };
            result->add(pos, glm::vec3(0.6f, 0.8f, 1.0f));
        }

        onRebuild();
    });

    centerView_.show();
    ImGui::SliderFloat("Radius", &radius_, 0.01f, 100.0f);
    ImGui::SliderInt  ("Count",  &count_,  100,   1000000);
    generateButton_.show();
}

} // namespace VPC
