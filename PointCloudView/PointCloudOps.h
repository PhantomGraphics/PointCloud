#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace VPC {

class World;

// Typed point-cloud processing functions shared by the GUI panels and the
// scenario CommandDispatcher, so both entry points produce byte-identical
// geometry, colour, visibility and selection
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md Phase 4). Migrated one
// operation at a time.
//
// Every op operates on the scene `activeSceneId` in `world`, adds one or more
// result scenes, usually hides the source, and reports the primary result
// scene id. The op does NOT change the active selection or trigger a renderer
// rebuild -- the caller applies ProcessOutcome (the GUI selects
// primarySceneId + rebuilds; the dispatcher does the same and maps failures to
// its "Error:<message>" string).
namespace ops {

struct ProcessOutcome {
    bool        ok = false;
    std::string message;        // human-readable; on failure, the reason (no "Error:" prefix)
    int         primarySceneId = -1;
};

// --- Down sample -----------------------------------------------------------

struct DownSampleParams {
    float cellSize = 0.05f;
};

ProcessOutcome downSample(World& world, int activeSceneId, const DownSampleParams& p);

// --- Normal estimation (plain + viewpoint-oriented) -----------------------

struct NormalParams {
    float     radius = 0.05f;
    bool      orientToViewpoint = false;
    glm::vec3 viewpoint{ 0.f };
};

// Result scene: "NormalResult" (orientToViewpoint == false) or
// "OrientedNormalResult" (true). Per-point colour = normal mapped to RGB via
// (n + 1) * 0.5; normals are stored on the result scene.
ProcessOutcome estimateNormals(World& world, int activeSceneId, const NormalParams& p);

// --- Density / curvature filters -----------------------------------------

struct DensityFilterParams {
    float radius = 0.05f;
};

// Result scene: "DensityFilterResult" -- the inlier points, colour (0.6,0.85,1).
ProcessOutcome filterDensity(World& world, int activeSceneId, const DensityFilterParams& p);

struct CurvatureFilterParams {
    float radius    = 0.01f;
    float threshold = 0.05f;
};

// Result scene: "CurvatureFilterResult" -- points with curvature <= threshold,
// grayscale colour = curvature / maxCurvature.
ProcessOutcome filterCurvature(World& world, int activeSceneId, const CurvatureFilterParams& p);

// --- RANSAC primitive detection ----------------------------------------

struct RansacParams {
    float threshold  = 0.02f;
    int   iterations = 200;
    int   minInliers = 50;
    bool  buildOverlayMesh = false;  // GUI adds a plane/cylinder polygon overlay
};

struct RansacPlaneResult {
    ProcessOutcome outcome;
    glm::vec3 normal{ 0.f };
    float     offset = 0.f;
    int       inlierCount = 0;
};
struct RansacCylinderResult {
    ProcessOutcome outcome;
    glm::vec3 axis{ 0.f };
    float     radius = 0.f;
    int       inlierCount = 0;
};
struct RansacSphereResult {
    ProcessOutcome outcome;
    glm::vec3 center{ 0.f };
    float     radius = 0.f;
    int       inlierCount = 0;
};
struct RansacConeResult {
    ProcessOutcome outcome;
    glm::vec3 apex{ 0.f };
    glm::vec3 axis{ 0.f };
    float     halfAngleRad = 0.f;
    int       inlierCount = 0;
};

// Each adds "<Shape>Inliers" (primary) + "<Shape>Outliers", hides the source.
RansacPlaneResult    detectPlane   (World& world, int activeSceneId, const RansacParams& p);
RansacCylinderResult detectCylinder(World& world, int activeSceneId, const RansacParams& p);
RansacSphereResult   detectSphere  (World& world, int activeSceneId, const RansacParams& p);
RansacConeResult     detectCone    (World& world, int activeSceneId, const RansacParams& p);

} // namespace ops
} // namespace VPC
