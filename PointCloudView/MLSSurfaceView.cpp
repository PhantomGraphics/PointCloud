#include "MLSSurfaceView.h"

#include "MLSSurface.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void MLSSurfaceView::onImGui(World& world, int activeSceneId,
                              const std::function<void()>& onRebuild)
{
    smoothButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        Phantom::PC::MLSSurface mls;
        for (const auto& p : scene->getPositions()) mls.add(p);
        mls.smooth(static_cast<double>(searchRadius_));
        const auto smoothed = mls.getSmoothedPoints();

        auto* result = world.addScene("MLSSmoothed");
        for (const auto& p : smoothed) result->add(p, glm::vec3(0.5f, 0.85f, 0.9f));

        scene->setVisible(false);
        onRebuild();
        status_ = "Smoothed " + std::to_string(smoothed.size()) + " points";
    });

    upsampleButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        Phantom::PC::MLSSurface mls;
        for (const auto& p : scene->getPositions()) mls.add(p);
        const auto upsampled = mls.upsample(static_cast<double>(searchRadius_), upsampleRadius_, stepSize_);
        if (upsampled.empty()) { status_ = "Upsample produced no points"; return; }

        auto* result = world.addScene("MLSUpsampled");
        for (const auto& p : upsampled) result->add(p, glm::vec3(0.9f, 0.7f, 0.9f));

        onRebuild();
        status_ = "Generated " + std::to_string(upsampled.size()) + " new points";
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderFloat("Upsample Radius", &upsampleRadius_, 0.001f, 0.5f, "%.4f");
    ImGui::SliderFloat("Step Size", &stepSize_, 0.001f, 0.2f, "%.4f");

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    smoothButton_.show();
    ImGui::SameLine();
    upsampleButton_.show();
}

} // namespace VPC
