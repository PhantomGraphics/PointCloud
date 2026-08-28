#include "PCRectView.h"

#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void PCRectView::onImGui(World& world, int /*activeSceneId*/,
                          const std::function<void()>& onRebuild)
{
    generateButton_.setFunction([&world, &onRebuild, this]() {
        if (uCount_ <= 0 || vCount_ <= 0) return;

        const auto ov = originView_.getValue();
        const auto uv = uvecView_.getValue();
        const auto vv = vvecView_.getValue();

        const glm::vec3 o{ ov.x, ov.y, ov.z };
        const glm::vec3 u{ uv.x, uv.y, uv.z };
        const glm::vec3 v{ vv.x, vv.y, vv.z };

        auto* result = world.addScene("Rect");
        for (int i = 0; i < uCount_; ++i) {
            const float ut = static_cast<float>(i) / static_cast<float>(uCount_ - 1);
            for (int j = 0; j < vCount_; ++j) {
                const float vt = static_cast<float>(j) / static_cast<float>(vCount_ - 1);
                glm::vec3 pos  = o + ut * u + vt * v;
                result->add(pos, glm::vec3(ut, vt, 1.0f));
            }
        }

        onRebuild();
    });

    originView_.show();
    uvecView_.show();
    vvecView_.show();
    ImGui::SliderInt  ("U Count", &uCount_, 2, 500);
    ImGui::SliderInt  ("V Count", &vCount_, 2, 500);
    generateButton_.show();
}

} // namespace VPC
