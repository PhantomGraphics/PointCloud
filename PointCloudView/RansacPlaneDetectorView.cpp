#include "RansacPlaneDetectorView.h"

#include "RansacPlaneDetector.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace VPC {

namespace {

// 2-D convex hull (Graham scan).
static std::vector<glm::vec2> convexHull(std::vector<glm::vec2> pts) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) return pts;

    // Move the bottom-most (then left-most) point to index 0.
    int pivot = 0;
    for (int i = 1; i < n; ++i) {
        if (pts[i].y < pts[pivot].y ||
            (pts[i].y == pts[pivot].y && pts[i].x < pts[pivot].x))
            pivot = i;
    }
    std::swap(pts[0], pts[pivot]);
    const glm::vec2 p0 = pts[0];

    std::sort(pts.begin() + 1, pts.end(), [&](const glm::vec2& a, const glm::vec2& b) {
        glm::vec2 da = a - p0, db = b - p0;
        float cross = da.x * db.y - da.y * db.x;
        if (std::abs(cross) > 1e-8f) return cross > 0.f;
        return glm::length(da) < glm::length(db);
    });

    std::vector<glm::vec2> hull;
    hull.reserve(n);
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2) {
            glm::vec2 a = hull[hull.size() - 2], b = hull[hull.size() - 1];
            glm::vec2 ab = b - a, bp = pts[i] - b;
            if (ab.x * bp.y - ab.y * bp.x <= 0.f)
                hull.pop_back();
            else
                break;
        }
        hull.push_back(pts[i]);
    }
    return hull;
}

// Build a plane polygon mesh on the inlier convex hull and add it to the world.
static void buildPlaneMesh(
    const std::vector<glm::vec3>& positions,
    const Phantom::PC::RansacPlaneDetector::PlaneModel& model,
    World& world)
{
    if (model.inliers.empty()) return;

    // Compute centroid.
    glm::vec3 centroid(0.f);
    for (size_t idx : model.inliers) centroid += positions[idx];
    centroid /= static_cast<float>(model.inliers.size());

    // Build orthonormal basis on the plane surface.
    const glm::vec3 normal = glm::normalize(glm::vec3(model.normal));
    const glm::vec3 arb = (std::abs(normal.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 uAxis = glm::normalize(glm::cross(normal, arb));
    const glm::vec3 vAxis = glm::cross(normal, uAxis);

    // Project inliers to 2-D.
    std::vector<glm::vec2> pts2d;
    pts2d.reserve(model.inliers.size());
    for (size_t idx : model.inliers) {
        glm::vec3 d = positions[idx] - centroid;
        pts2d.push_back({ glm::dot(d, uAxis), glm::dot(d, vAxis) });
    }

    // Convex hull in 2-D.
    auto hull2d = convexHull(pts2d);
    if (hull2d.size() < 3) return;

    PolygonMesh mesh;
    mesh.name = "PlanePolygon";

    constexpr float r = 0.2f, g = 0.9f, b = 0.3f, a = 0.5f;

    for (const auto& p2 : hull2d) {
        glm::vec3 p3 = centroid + p2.x * uAxis + p2.y * vAxis;
        mesh.positions.insert(mesh.positions.end(), { p3.x, p3.y, p3.z });
        mesh.colors.insert(mesh.colors.end(), { r, g, b, a });
    }

    // Fan triangulation.
    for (size_t i = 1; i + 1 < hull2d.size(); ++i) {
        mesh.indices.push_back(0);
        mesh.indices.push_back(static_cast<uint32_t>(i));
        mesh.indices.push_back(static_cast<uint32_t>(i + 1));
    }

    world.clearPolygons();
    world.addPolygon(std::move(mesh));
}

} // namespace

void RansacPlaneDetectorView::onImGui(World& world, int activeSceneId,
                                       const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();

        Phantom::PC::RansacPlaneDetector detector;
        Phantom::PC::RansacPlaneDetector::PlaneModel model;

        const bool ok = detector.detect(
            positions, model, iterations_, threshold_,
            static_cast<size_t>(minInliers_));

        if (!ok) { status_ = "Detection failed"; return; }

        std::vector<bool> isInlier(positions.size(), false);

        auto* inliers = world.addScene("PlaneInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.2f, 0.9f, 0.3f));
            }
        }

        auto* outliers = world.addScene("PlaneOutliers");
        for (size_t i = 0; i < positions.size(); ++i) {
            if (!isInlier[i]) {
                outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
            }
        }

        buildPlaneMesh(positions, model, world);

        scene->setVisible(false);
        onRebuild();

        hasResult_   = true;
        succeeded_   = true;
        nx_ = model.normal.x; ny_ = model.normal.y; nz_ = model.normal.z;
        offset_      = model.offset;
        inlierCount_ = static_cast<int>(model.inliers.size());
        status_      = "Plane detected";
    });

    ImGui::SliderFloat("Threshold",   &threshold_,  0.001f, 0.5f,  "%.4f");
    ImGui::SliderInt  ("Iterations",  &iterations_, 10,     1000);
    ImGui::SliderInt  ("Min Inliers", &minInliers_, 3,      5000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Normal: (%.5f, %.5f, %.5f)", nx_, ny_, nz_);
        ImGui::Text("Offset: %.5f", offset_);
        ImGui::Text("Inliers: %d",  inlierCount_);
    }
    runButton_.show();
}

} // namespace VPC
