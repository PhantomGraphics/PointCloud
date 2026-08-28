#include "ConcaveHull2DView.h"

#include "ConcaveHull2D.h"
#include "imgui.h"

#include <algorithm>
#include <glm/glm.hpp>

namespace VPC {

void ConcaveHull2DView::onImGui(World& world, int activeSceneId,
                                 const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        Phantom::PC::ConcaveHull2D hull;
        for (const auto& p : scene->getPositions()) hull.add(p);
        if (!hull.compute(static_cast<size_t>(std::max(3, k_)), static_cast<size_t>(std::max(0, maxK_)))) {
            status_ = "Concave hull failed (need >= 3 distinct points)";
            return;
        }

        const auto verts = hull.getHullPoints();

        auto* result = world.addScene("ConcaveHullVertices");
        for (const auto& v : verts) result->add(v, glm::vec3(0.9f, 0.6f, 0.3f));

        world.clearPolygons();
        PolygonMesh mesh;
        mesh.name = "ConcaveHullPolygon";
        constexpr float r = 0.9f, g = 0.6f, b = 0.3f, a = 0.5f;
        for (const auto& v : verts) {
            mesh.positions.insert(mesh.positions.end(), { v.x, v.y, v.z });
            mesh.colors.insert(mesh.colors.end(), { r, g, b, a });
        }
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            mesh.indices.push_back(0);
            mesh.indices.push_back(static_cast<uint32_t>(i));
            mesh.indices.push_back(static_cast<uint32_t>(i + 1));
        }
        world.addPolygon(std::move(mesh));

        scene->setVisible(false);
        onRebuild();

        hasResult_   = true;
        succeeded_   = true;
        area_        = hull.getArea();
        vertexCount_ = static_cast<int>(verts.size());
        status_      = "Hull computed";
    });

    ImGui::Text("Computes the concave hull of the active scene's XY projection.");
    ImGui::SliderInt("Initial k", &k_, 3, 30);
    ImGui::SliderInt("Max k (0 = unbounded)", &maxK_, 0, 200);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Vertices: %d", vertexCount_);
        ImGui::Text("Area: %.5f", area_);
    }
    runButton_.show();
}

} // namespace VPC
