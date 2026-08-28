#include "GroundExtractorView.h"

#include "GroundExtractor.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void GroundExtractorView::onImGui(World& world, int activeSceneId,
                                   const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();

        Phantom::PC::GroundExtractor extractor;
        for (const auto& p : positions) extractor.add(p);

        Phantom::PC::GroundExtractor::Params params;
        params.cellSize                  = cellSize_;
        params.slope                     = slope_;
        params.initialWindowSize         = initialWindowSize_;
        params.maxWindowSize             = maxWindowSize_;
        params.windowGrowthFactor        = windowGrowthFactor_;
        params.initialElevationThreshold = initialElevationThreshold_;
        params.maxElevationThreshold     = maxElevationThreshold_;
        params.finalElevationThreshold   = finalElevationThreshold_;

        if (!extractor.extract(params)) { status_ = "Extraction failed"; return; }
        const auto flags = extractor.getGroundFlags();

        auto* ground    = world.addScene("GroundPoints");
        auto* nonGround = world.addScene("NonGroundPoints");
        int gCount = 0, ngCount = 0;
        for (size_t i = 0; i < positions.size() && i < flags.size(); ++i) {
            if (flags[i]) { ground->add(positions[i], glm::vec3(0.4f, 0.8f, 0.3f)); ++gCount; }
            else          { nonGround->add(positions[i], glm::vec3(0.7f, 0.45f, 0.2f)); ++ngCount; }
        }

        scene->setVisible(false);
        onRebuild();

        hasResult_      = true;
        succeeded_      = true;
        groundCount_    = gCount;
        nonGroundCount_ = ngCount;
        status_         = "Extraction complete";
    });

    ImGui::SliderFloat("Cell Size", &cellSize_, 0.05f, 10.0f, "%.3f");
    ImGui::SliderFloat("Slope", &slope_, 0.01f, 2.0f, "%.3f");

    if (ImGui::CollapsingHeader("Advanced")) {
        ImGui::SliderFloat("Initial Window Size", &initialWindowSize_, 0.1f, 10.0f, "%.3f");
        ImGui::SliderFloat("Max Window Size", &maxWindowSize_, 1.0f, 64.0f, "%.2f");
        ImGui::SliderFloat("Window Growth Factor", &windowGrowthFactor_, 1.01f, 4.0f, "%.3f");
        ImGui::SliderFloat("Initial Elevation Threshold", &initialElevationThreshold_, 0.01f, 5.0f, "%.3f");
        ImGui::SliderFloat("Max Elevation Threshold", &maxElevationThreshold_, 0.1f, 20.0f, "%.3f");
        ImGui::SliderFloat("Final Elevation Threshold", &finalElevationThreshold_, 0.01f, 5.0f, "%.3f");
    }

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Ground Points: %d", groundCount_);
        ImGui::Text("Non-Ground Points: %d", nonGroundCount_);
    }
    runButton_.show();
}

} // namespace VPC
