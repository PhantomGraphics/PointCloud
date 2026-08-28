#include "PoissonSurfaceView.h"

#include "PoissonSurface.h"
#include "imgui.h"

namespace VPC {

void PoissonSurfaceView::onImGui(World& world, int activeSceneId,
                                  const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        vertexCount_   = 0;
        triangleCount_ = 0;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }
        if (!scene->hasNormals()) { status_ = "Scene has no normals (run Normal Estimator first)"; return; }

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();
        if (positions.empty()) { status_ = "No points in scene"; return; }

        std::vector<PSPoint> pts;
        pts.reserve(positions.size());
        const size_t n = std::min(positions.size(), normals.size());
        for (size_t i = 0; i < n; ++i) {
            PSPoint p;
            p.x = positions[i].x; p.y = positions[i].y; p.z = positions[i].z;
            p.nx = normals[i].x;  p.ny = normals[i].y;  p.nz = normals[i].z;
            pts.push_back(p);
        }

        PoissonSurface::Config cfg;
        cfg.resolution    = resolution_;
        cfg.samplesPerCell = samplesPerCell_;
        cfg.isoLevel      = isoLevel_;
        cfg.maxIters      = maxIters_;
        cfg.omega         = omega_;
        cfg.bboxPadding   = bboxPadding_;
        cfg.clampToBBox   = clampToBBox_;

        PoissonSurface solver(cfg);
        const PSTriangleMesh result = solver.reconstruct(pts);

        if (result.faces.empty()) { status_ = "Reconstruction produced no triangles"; return; }

        PolygonMesh mesh;
        mesh.name = "PoissonMesh";

        constexpr float r = 1.0f, g = 0.7f, b = 0.3f, a = 0.8f;

        for (const auto& v : result.vertices) {
            mesh.positions.insert(mesh.positions.end(), { v[0], v[1], v[2] });
            mesh.colors.insert(mesh.colors.end(), { r, g, b, a });
        }
        for (const auto& f : result.faces) {
            mesh.indices.push_back(f[0]);
            mesh.indices.push_back(f[1]);
            mesh.indices.push_back(f[2]);
        }

        world.clearPolygons();
        world.addPolygon(std::move(mesh));

        vertexCount_   = static_cast<int>(result.vertices.size());
        triangleCount_ = static_cast<int>(result.faces.size());
        status_ = "Done";
        onRebuild();
    });

    ImGui::SliderInt  ("Resolution",      &resolution_,     32,   256);
    ImGui::SliderFloat("Samples/Cell",    &samplesPerCell_, 0.5f,  4.0f, "%.2f");
    ImGui::SliderFloat("Iso Level",       &isoLevel_,      -1.0f, 1.0f, "%.3f");
    ImGui::SliderInt  ("Max Iterations",  &maxIters_,       50,   500);
    ImGui::SliderFloat("SOR Omega",       &omega_,          1.0f,  1.99f, "%.2f");
    ImGui::SliderFloat("BBox Padding",    &bboxPadding_,    0.5f,  5.0f, "%.1f");
    ImGui::Checkbox   ("Clamp to BBox",   &clampToBBox_);

    ImGui::Spacing();
    ImGui::TextDisabled("Requires normals (run Normal Estimator first).");
    ImGui::Text("Status: %s", status_.c_str());
    if (triangleCount_ > 0) {
        ImGui::Text("Vertices: %d  Triangles: %d", vertexCount_, triangleCount_);
    }
    runButton_.show();
}

} // namespace VPC
