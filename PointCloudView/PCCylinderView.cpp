#include "PCCylinderView.h"

#include "imgui.h"

#include <cmath>
#include <glm/glm.hpp>
#include <random>

namespace VPC {

void PCCylinderView::onImGui(World& world, int /*activeSceneId*/,
                              const std::function<void(int)>& onResult)
{
    generateButton_.setFunction([&world, &onResult, this]() {
        if (radius_ <= 0.0f || height_ <= 0.0f || count_ <= 0) return;

        std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);

        const auto center = centerView_.getValue();
        const glm::vec3 cx{ center.x, center.y, center.z };

        auto* result = world.addScene("Cylinder");
        for (int i = 0; i < count_; ++i) {
            const float u     = dist(rng);
            const float v     = dist(rng);
            const float theta = 2.0f * 3.14159265f * u;
            glm::vec3 pos = cx + glm::vec3{
                radius_ * std::cosf(theta),
                height_ * v,
                radius_ * std::sinf(theta)
            };
            result->add(pos, glm::vec3(0.8f, 0.6f, 1.0f));
        }

        onResult(-1);
    });

    centerView_.show();
    ImGui::SliderFloat("Radius", &radius_, 0.01f, 100.0f);
    ImGui::SliderFloat("Height", &height_, 0.01f, 100.0f);
    ImGui::SliderInt  ("Count",  &count_,  100,   1000000);
    generateButton_.show();
}

} // namespace VPC
