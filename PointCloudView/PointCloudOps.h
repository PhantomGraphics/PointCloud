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

// --- Clustering / segmentation -----------------------------------------

struct DbscanParams   { float eps = 0.02f; int minPts = 10; };
struct DistanceClusterParams { float radius = 0.02f; };
struct RegionGrowParams {
    float curvatureRadius = 0.1f;
    int   kNeighbors      = 30;
    float smoothnessDeg   = 5.0f;
    float curvatureThreshold = 1.0f;
    int   minClusterSize  = 10;
};

struct ClusterResult {
    ProcessOutcome outcome;
    int clusterCount = 0;
};

// "DBSCANResult" -- per-cluster colour (grey for noise), source hidden.
ClusterResult clusterDbscan(World& world, int activeSceneId, const DbscanParams& p);
// "RegionGrowingResult" -- distance-based region growing, per-cluster colour.
ClusterResult clusterDistance(World& world, int activeSceneId, const DistanceClusterParams& p);
// "RegionGrowingNormalResult" -- normal/curvature region growing (needs normals),
// per-cluster colour; curvature is recomputed from curvatureRadius.
ClusterResult segmentRegionGrowing(World& world, int activeSceneId, const RegionGrowParams& p);

// --- Ground extraction ------------------------------------------------

struct GroundParams {
    float cellSize                  = 1.0f;
    float slope                     = 0.3f;
    float initialWindowSize         = 1.0f;
    float maxWindowSize             = 16.0f;
    float windowGrowthFactor        = 2.0f;
    float initialElevationThreshold = 0.2f;
    float maxElevationThreshold     = 3.0f;
    float finalElevationThreshold   = 0.3f;
};

struct GroundResult {
    ProcessOutcome outcome;   // primarySceneId = "GroundPoints"
    int   groundCount = 0;
    int   nonGroundCount = 0;
    double groundRatio = 0.0;
};

GroundResult extractGround(World& world, int activeSceneId, const GroundParams& p);

// --- Curvature / FPFH / boundary ------------------------------------

struct CurvatureParams {
    float radius    = 0.05f;
    bool  principal = false;   // false: scalar (PCA eigenvalue ratio); true: k1/k2
};

struct CurvatureResult {
    ProcessOutcome outcome;    // "CurvatureResult" or "PrincipalCurvatureResult"
    double meanK1 = 0.0;       // valid iff principal
    double meanK2 = 0.0;
};

CurvatureResult estimateCurvature(World& world, int activeSceneId, const CurvatureParams& p);

struct FpfhParams { int kNeighbors = 20; };
struct FpfhResult {
    ProcessOutcome outcome;    // "FPFHResult" (needs normals)
    int descriptorCount = 0;
    int histogramSize   = 0;
};
FpfhResult estimateFpfh(World& world, int activeSceneId, const FpfhParams& p);

struct BoundaryParams { float radius = 0.1f; float angleThresholdDeg = 153.0f; };
struct BoundaryResult {
    ProcessOutcome outcome;    // "BoundaryResult" (needs normals)
    int boundaryCount = 0;
};
BoundaryResult detectBoundary(World& world, int activeSceneId, const BoundaryParams& p);

// --- MLS surface -----------------------------------------------------

struct MlsSmoothParams { float radius = 0.1f; };
struct MlsUpsampleParams {
    float radius         = 0.1f;
    float upsampleRadius = 0.05f;
    float stepSize       = 0.01f;
};

// "MLSSmoothed" (colour 0.5,0.85,0.9), source hidden.
ProcessOutcome mlsSmooth(World& world, int activeSceneId, const MlsSmoothParams& p);
// "MLSUpsampled" (colour 0.9,0.7,0.9). Source stays visible (new points are added).
ProcessOutcome mlsUpsample(World& world, int activeSceneId, const MlsUpsampleParams& p);

// --- 2D hulls -------------------------------------------------------

struct ConcaveHullParams { int k = 3; int maxK = 0; }; // maxK 0 = unbounded

struct HullResult {
    ProcessOutcome outcome;   // "<Convex|Concave>HullVertices" + a "<...>Polygon" overlay
    double area = 0.0;
    int    vertexCount = 0;
};

HullResult convexHull2D (World& world, int activeSceneId);
HullResult concaveHull2D(World& world, int activeSceneId, const ConcaveHullParams& p);

// --- Registration -------------------------------------------------

struct IcpParams {
    int   targetSceneId = -1;
    bool  pointToPlane  = false;
    int   maxIterations = 50;
    float tolerance     = 1.0e-6f;
    float maxCorrespondenceDistance = 0.0f;
    int   robustKernel  = 0;   // 0 None, 1 Huber, 2 Tukey
    float robustKernelDelta = 1.0f;
    bool  estimateScale = false;
};

struct IcpResult {
    ProcessOutcome outcome;   // "ICPAligned" / "ICPAlignedP2Plane"
    float fitness = 0.f;
    int   iterations = 0;
    bool  converged = false;
    float scale = 1.f;
};

IcpResult icpAlign(World& world, int sourceSceneId, const IcpParams& p);

struct GlobalRegisterParams {
    int   targetSceneId  = -1;
    int   fpfhK          = 20;
    int   iterations     = 1000;
    float maxCorrespondenceDistance = 0.05f;
    int   sampleSize     = 3;
    float edgeLengthTolerance = 0.15f;
    int   minInliers     = 3;
};

struct GlobalRegisterResult {
    ProcessOutcome outcome;   // "GlobalRegAligned"
    int    inlierCount = 0;
    double inlierRmse  = 0.0;
};

GlobalRegisterResult globalRegister(World& world, int sourceSceneId, const GlobalRegisterParams& p);

} // namespace ops
} // namespace VPC
