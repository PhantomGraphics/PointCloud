#include "ConvexHull2DView.h"

#include "ConvexHull2D.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void ConvexHull2DView::onImGui(World& world, int activeSceneId,
                                const std::function<void(int)>& onResult)
{
    runButton_.setFunction([&world, activeSceneId, &onResult, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        Phantom::PC::ConvexHull2D hull;
        for (const auto& p : scene->getPositions()) hull.add(p);
        if (!hull.compute()) { status_ = "Convex hull failed (need >= 3 non-collinear points)"; return; }

        const auto verts = hull.getHullPoints();

        auto* result = world.addScene("ConvexHullVertices");
        for (const auto& v : verts) result->add(v, glm::vec3(0.3f, 0.9f, 0.6f));

        world.clearPolygons();
        PolygonMesh mesh;
        mesh.name = "ConvexHullPolygon";
        constexpr float r = 0.3f, g = 0.9f, b = 0.6f, a = 0.5f;
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
        onResult(-1);

        hasResult_   = true;
        succeeded_   = true;
        area_        = hull.getArea();
        vertexCount_ = static_cast<int>(verts.size());
        status_      = "Hull computed";
    });

    ImGui::Text("Computes the convex hull of the active scene's XY projection.");

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Vertices: %d", vertexCount_);
        ImGui::Text("Area: %.5f", area_);
    }
    runButton_.show();
}

} // namespace VPC
