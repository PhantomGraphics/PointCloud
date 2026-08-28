#include "DBSCANClusteringView.h"

#include "DBSCAN.h"
#include "imgui.h"

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
    if (id <= 0) return { 0.5f, 0.5f, 0.5f };
    const float hue = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f);
    return hsvToRgb(hue, 0.85f, 1.0f);
}

} // namespace

void DBSCANClusteringView::onImGui(World& world, int activeSceneId,
                                    const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr || eps_ <= 0.0f || minPts_ <= 0) return;

        const auto& positions = scene->getPositions();

        std::vector<Phantom::PC::Point> pts;
        pts.reserve(positions.size());
        for (const auto& p : positions) {
            pts.emplace_back(
                static_cast<double>(p.x),
                static_cast<double>(p.y),
                static_cast<double>(p.z));
        }

        Phantom::PC::DBSCANClustering clustering;
        clustering.cluster(pts, eps_, minPts_);

        auto* result = world.addScene("DBSCANResult");
        for (size_t i = 0; i < positions.size(); ++i) {
            result->add(positions[i], colorFromCluster(pts[i].clusterID));
        }

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Eps",     &eps_,    0.001f, 1.0f, "%.4f");
    ImGui::SliderInt  ("Min Pts", &minPts_, 1,      100);
    runButton_.show();
}

} // namespace VPC
