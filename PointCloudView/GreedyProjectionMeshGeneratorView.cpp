#include "GreedyProjectionMeshGeneratorView.h"

#include "GreedyProjectionMeshGenerator.h"
#include "imgui.h"

namespace VPC {

void GreedyProjectionMeshGeneratorView::onImGui(World& world, int activeSceneId,
                                                 const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        triangleCount_ = 0;
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();
        if (positions.empty()) { status_ = "No points in scene"; return; }

        Phantom::PC::GreedyProjectionMeshGenerator generator;
        generator.generate(positions);

        const auto& triangles = generator.getTriangles();
        if (triangles.empty()) { status_ = "No triangles generated"; return; }

        PolygonMesh mesh;
        mesh.name = "GreedyMesh";

        constexpr float r = 0.3f, g = 0.6f, b = 1.0f, a = 0.7f;

        for (const auto& tri : triangles) {
            const auto verts = tri.getVertices();
            uint32_t base = static_cast<uint32_t>(mesh.positions.size() / 3);
            for (const auto& v : verts) {
                mesh.positions.insert(mesh.positions.end(), { v.x, v.y, v.z });
                mesh.colors.insert(mesh.colors.end(), { r, g, b, a });
            }
            mesh.indices.push_back(base);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 2);
        }

        world.clearPolygons();
        world.addPolygon(std::move(mesh));

        triangleCount_ = static_cast<int>(triangles.size());
        status_ = "Done";
        onRebuild();
    });

    ImGui::Text("Points will be meshed using greedy projection.");
    ImGui::Spacing();
    ImGui::Text("Status: %s", status_.c_str());
    if (triangleCount_ > 0)
        ImGui::Text("Triangles: %d", triangleCount_);
    runButton_.show();
}

} // namespace VPC
