#include "RegionGrowingView.h"

#include "CurvatureEstimator.h"
#include "RegionGrowing.h"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <vector>

namespace VPC {

namespace {

glm::vec3 hsvToRgb(float h, float s, float v)
{
    const float c  = v * s;
    const float hh = h * 6.0f;
    const float x  = c * (1.0f - std::fabs(std::fmod(hh, 2.0f) - 1.0f));
    const float m  = v - c;

    if (hh < 1.0f) return { c + m, x + m, m };
    if (hh < 2.0f) return { x + m, c + m, m };
    if (hh < 3.0f) return { m, c + m, x + m };
    if (hh < 4.0f) return { m, x + m, c + m };
    if (hh < 5.0f) return { x + m, m, c + m };
    return { c + m, m, x + m };
}

glm::vec3 colorFromCluster(int id)
{
    if (id < 0) return { 0.5f, 0.5f, 0.5f };
    const float hue = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f);
    return hsvToRgb(hue, 0.85f, 1.0f);
}

} // namespace

void RegionGrowingView::onImGui(World& world, int activeSceneId,
                                 const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }
        if (!scene->hasNormals()) { status_ = "Scene has no normals - run Normal Estimator first"; return; }

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        // PointCloudfScene doesn't persist curvature, so it's recomputed here from the same
        // radius used for the neighborhood search (see docs/todo/PLAN_pointcloudview_new_algorithm_views.md §4).
        Phantom::PC::CurvatureEstimator curvEstimator;
        for (const auto& p : positions) curvEstimator.add(p);
        curvEstimator.estimate(static_cast<double>(curvatureRadius_));
        const auto curvatures = curvEstimator.getCurvatures();

        Phantom::PC::RegionGrowing regionGrowing;
        for (size_t i = 0; i < positions.size(); ++i)
            regionGrowing.add(positions[i], normals[i], curvatures[i]);

        Phantom::PC::RegionGrowing::Params params;
        params.kNeighbors = static_cast<size_t>(std::max(1, kNeighbors_));
        params.smoothnessThresholdRad = smoothnessThresholdDeg_ * 3.14159265f / 180.0f;
        params.curvatureThreshold = static_cast<double>(curvatureThreshold_);
        params.minClusterSize = static_cast<size_t>(std::max(1, minClusterSize_));

        if (!regionGrowing.segment(params)) { status_ = "Region growing failed"; return; }

        const auto labels = regionGrowing.getLabels();
        auto* result = world.addScene("RegionGrowingNormalResult");
        for (size_t i = 0; i < positions.size(); ++i)
            result->add(positions[i], colorFromCluster(labels[i]));

        scene->setVisible(false);
        onRebuild();

        hasResult_    = true;
        succeeded_    = true;
        clusterCount_ = regionGrowing.getClusterCount();
        status_       = "Segmentation complete";
    });

    ImGui::SliderFloat("Curvature Radius", &curvatureRadius_, 0.001f, 1.0f, "%.4f");
    ImGui::SliderInt  ("k Neighbors", &kNeighbors_, 5, 100);
    ImGui::SliderFloat("Smoothness Threshold (deg)", &smoothnessThresholdDeg_, 0.5f, 45.0f, "%.2f");
    ImGui::SliderFloat("Curvature Threshold", &curvatureThreshold_, 0.001f, 5.0f, "%.4f");
    ImGui::SliderInt  ("Min Cluster Size", &minClusterSize_, 1, 1000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Cluster Count: %zu", clusterCount_);
    }
    runButton_.show();
}

} // namespace VPC
