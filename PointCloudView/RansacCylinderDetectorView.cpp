#include "RansacCylinderDetectorView.h"

#include "RansacCylinderDetector.h"
#include "imgui.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace VPC {

namespace {

// Build a cylinder mesh and add it to the world.
static void buildCylinderMesh(
    const std::vector<glm::vec3>& positions,
    const Phantom::PC::RansacCylinderDetector::CylinderModel& model,
    World& world)
{
    if (model.inliers.empty()) return;

    const glm::vec3 origin    = glm::vec3(model.origin);
    const glm::vec3 direction = glm::normalize(glm::vec3(model.direction));
    const float     radius    = model.radius;

    // Compute the extent of inliers along the cylinder axis.
    float tMin = std::numeric_limits<float>::max();
    float tMax = std::numeric_limits<float>::lowest();
    for (size_t idx : model.inliers) {
        float t = glm::dot(positions[idx] - origin, direction);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }
    if (tMax <= tMin) tMax = tMin + 0.01f;

    // Build orthonormal basis perpendicular to the axis.
    const glm::vec3 arb   = (std::abs(direction.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 rAxis = glm::normalize(glm::cross(direction, arb));
    const glm::vec3 sAxis = glm::cross(direction, rAxis);

    constexpr int   N     = 32;
    const float     dTheta = glm::two_pi<float>() / N;

    auto ringPos = [&](float t, int i) -> glm::vec3 {
        float theta = i * dTheta;
        return origin + t * direction + radius * (std::cos(theta) * rAxis + std::sin(theta) * sAxis);
    };

    PolygonMesh mesh;
    mesh.name = "CylinderPolygon";

    constexpr float cr = 0.2f, cg = 0.7f, cb = 1.0f, ca = 0.5f;

    auto pushVert = [&](const glm::vec3& p) {
        mesh.positions.insert(mesh.positions.end(), { p.x, p.y, p.z });
        mesh.colors.insert(mesh.colors.end(), { cr, cg, cb, ca });
    };

    // Bottom center (index 0)
    pushVert(origin + tMin * direction);
    // Bottom ring (index 1..N)
    for (int i = 0; i < N; ++i) pushVert(ringPos(tMin, i));
    // Top center (index N+1)
    pushVert(origin + tMax * direction);
    // Top ring (index N+2..2N+1)
    for (int i = 0; i < N; ++i) pushVert(ringPos(tMax, i));

    const uint32_t botCenter = 0;
    const uint32_t topCenter = static_cast<uint32_t>(N + 1);

    for (int i = 0; i < N; ++i) {
        uint32_t b0 = 1u + i;
        uint32_t b1 = 1u + (i + 1) % N;
        uint32_t t0 = static_cast<uint32_t>(N + 2) + i;
        uint32_t t1 = static_cast<uint32_t>(N + 2) + (i + 1) % N;

        // Bottom cap (CCW: b1, b0)
        mesh.indices.insert(mesh.indices.end(), { botCenter, b1, b0 });
        // Side (2 triangles)
        mesh.indices.insert(mesh.indices.end(), { b0, b1, t0 });
        mesh.indices.insert(mesh.indices.end(), { t0, b1, t1 });
        // Top cap (CW: t0, t1)
        mesh.indices.insert(mesh.indices.end(), { topCenter, t0, t1 });
    }

    world.clearPolygons();
    world.addPolygon(std::move(mesh));
}

} // namespace

void RansacCylinderDetectorView::onImGui(World& world, int activeSceneId,
                                          const std::function<void(int)>& onResult)
{
    runButton_.setFunction([&world, activeSceneId, &onResult, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }

        const auto& positions = scene->getPositions();

        Phantom::PC::RansacCylinderDetector detector;
        Phantom::PC::RansacCylinderDetector::CylinderModel model;

        const bool ok = detector.detect(
            positions, model, iterations_, threshold_,
            static_cast<size_t>(minInliers_));

        if (!ok) { status_ = "Detection failed"; return; }

        std::vector<bool> isInlier(positions.size(), false);

        auto* inliers = world.addScene("CylinderInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.2f, 0.7f, 1.0f));
            }
        }

        auto* outliers = world.addScene("CylinderOutliers");
        for (size_t i = 0; i < positions.size(); ++i) {
            if (!isInlier[i]) {
                outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
            }
        }

        buildCylinderMesh(positions, model, world);

        scene->setVisible(false);
        onResult(-1);

        hasResult_   = true;
        succeeded_   = true;
        ax_ = model.direction.x; ay_ = model.direction.y; az_ = model.direction.z;
        radius_      = model.radius;
        inlierCount_ = static_cast<int>(model.inliers.size());
        status_      = "Cylinder detected";
    });

    ImGui::SliderFloat("Threshold",   &threshold_,  0.001f, 0.5f,  "%.4f");
    ImGui::SliderInt  ("Iterations",  &iterations_, 10,     1000);
    ImGui::SliderInt  ("Min Inliers", &minInliers_, 3,      5000);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Axis: (%.5f, %.5f, %.5f)", ax_, ay_, az_);
        ImGui::Text("Radius: %.5f", radius_);
        ImGui::Text("Inliers: %d",  inlierCount_);
    }
    runButton_.show();
}

} // namespace VPC
