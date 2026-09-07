#include "PointCloudOps.h"

#include "World.h"
#include "PointCloudfScene.h"

#include "DownSampler.h"
#include "NormalEstimator.h"
#include "DensityBasedFilter.h"
#include "CurvatureEstimator.h"
#include "RansacPlaneDetector.h"
#include "RansacCylinderDetector.h"
#include "RansacSphereDetector.h"
#include "RansacConeDetector.h"
#include "DBSCAN.h"
#include "DistanceBasedClustering.h"
#include "RegionGrowing.h"
#include "GroundExtractor.h"
#include "FPFHEstimator.h"
#include "BoundaryDetector.h"

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace VPC {
namespace ops {

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

glm::vec3 colorFromClusterId(int id)
{
    if (id < 0) return { 0.5f, 0.5f, 0.5f };
    const float hue = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f);
    return hsvToRgb(hue, 0.85f, 1.0f);
}

std::vector<Phantom::PC::Point> toPoints(const std::vector<glm::vec3>& positions)
{
    std::vector<Phantom::PC::Point> pts;
    pts.reserve(positions.size());
    for (const auto& p : positions)
        pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                         static_cast<double>(p.z));
    return pts;
}

// Common RANSAC front end: validate, run the detector's detect(), split the
// active scene into "<name>Inliers" (primary) + "<name>Outliers", hide the
// source. Templated on the detector + model type.
template <class Detector, class Model>
ProcessOutcome runRansac(World& world, int activeSceneId, const RansacParams& p,
                         const char* shape, const glm::vec3& inlierColour,
                         Model& modelOut)
{
    ProcessOutcome out;
    auto* scene = world.findById(activeSceneId);
    if (!scene) { out.message = "no active scene"; return out; }

    const auto& positions = scene->getPositions();

    Detector detector;
    if (!detector.detect(positions, modelOut, p.iterations, p.threshold,
                         static_cast<size_t>(p.minInliers))) {
        out.message = std::string(shape) + " detection failed";
        return out;
    }

    std::vector<bool> isInlier(positions.size(), false);
    auto* inliers = world.addScene(std::string(shape) + "Inliers");
    for (const auto idx : modelOut.inliers) {
        if (idx < positions.size()) {
            isInlier[idx] = true;
            inliers->add(positions[idx], inlierColour);
        }
    }
    auto* outliers = world.addScene(std::string(shape) + "Outliers");
    for (size_t i = 0; i < positions.size(); ++i)
        if (!isInlier[i]) outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));

    scene->setVisible(false);

    out.ok = true;
    out.primarySceneId = inliers->getId();
    return out;
}

// 2-D convex hull (Graham scan) -- for the plane overlay polygon.
std::vector<glm::vec2> convexHull(std::vector<glm::vec2> pts)
{
    const int n = static_cast<int>(pts.size());
    if (n < 3) return pts;

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
            if (ab.x * bp.y - ab.y * bp.x <= 0.f) hull.pop_back();
            else break;
        }
        hull.push_back(pts[i]);
    }
    return hull;
}

void buildPlaneMesh(const std::vector<glm::vec3>& positions,
                    const Phantom::PC::RansacPlaneDetector::PlaneModel& model,
                    World& world)
{
    if (model.inliers.empty()) return;

    glm::vec3 centroid(0.f);
    for (size_t idx : model.inliers) centroid += positions[idx];
    centroid /= static_cast<float>(model.inliers.size());

    const glm::vec3 normal = glm::normalize(glm::vec3(model.normal));
    const glm::vec3 arb = (std::abs(normal.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 uAxis = glm::normalize(glm::cross(normal, arb));
    const glm::vec3 vAxis = glm::cross(normal, uAxis);

    std::vector<glm::vec2> pts2d;
    pts2d.reserve(model.inliers.size());
    for (size_t idx : model.inliers) {
        glm::vec3 d = positions[idx] - centroid;
        pts2d.push_back({ glm::dot(d, uAxis), glm::dot(d, vAxis) });
    }

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
    for (size_t i = 1; i + 1 < hull2d.size(); ++i) {
        mesh.indices.push_back(0);
        mesh.indices.push_back(static_cast<uint32_t>(i));
        mesh.indices.push_back(static_cast<uint32_t>(i + 1));
    }
    world.clearPolygons();
    world.addPolygon(std::move(mesh));
}

void buildCylinderMesh(const std::vector<glm::vec3>& positions,
                       const Phantom::PC::RansacCylinderDetector::CylinderModel& model,
                       World& world)
{
    if (model.inliers.empty()) return;

    const glm::vec3 origin    = glm::vec3(model.origin);
    const glm::vec3 direction = glm::normalize(glm::vec3(model.direction));
    const float     radius    = model.radius;

    float tMin = std::numeric_limits<float>::max();
    float tMax = std::numeric_limits<float>::lowest();
    for (size_t idx : model.inliers) {
        float t = glm::dot(positions[idx] - origin, direction);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }
    if (tMax <= tMin) tMax = tMin + 0.01f;

    const glm::vec3 arb   = (std::abs(direction.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 rAxis = glm::normalize(glm::cross(direction, arb));
    const glm::vec3 sAxis = glm::cross(direction, rAxis);

    constexpr int N = 32;
    const float dTheta = glm::two_pi<float>() / N;
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

    pushVert(origin + tMin * direction);
    for (int i = 0; i < N; ++i) pushVert(ringPos(tMin, i));
    pushVert(origin + tMax * direction);
    for (int i = 0; i < N; ++i) pushVert(ringPos(tMax, i));

    const uint32_t botCenter = 0;
    const uint32_t topCenter = static_cast<uint32_t>(N + 1);
    for (int i = 0; i < N; ++i) {
        uint32_t b0 = 1u + i;
        uint32_t b1 = 1u + (i + 1) % N;
        uint32_t t0 = static_cast<uint32_t>(N + 2) + i;
        uint32_t t1 = static_cast<uint32_t>(N + 2) + (i + 1) % N;
        mesh.indices.insert(mesh.indices.end(), { botCenter, b1, b0 });
        mesh.indices.insert(mesh.indices.end(), { b0, b1, t0 });
        mesh.indices.insert(mesh.indices.end(), { t0, b1, t1 });
        mesh.indices.insert(mesh.indices.end(), { topCenter, t0, t1 });
    }
    world.clearPolygons();
    world.addPolygon(std::move(mesh));
}

} // namespace

// ------------------------------------------------------------------------

ProcessOutcome downSample(World& world, int activeSceneId, const DownSampleParams& p)
{
    ProcessOutcome out;

    auto* scene = world.findById(activeSceneId);
    if (!scene)            { out.message = "no active scene";  return out; }
    if (p.cellSize <= 0.f) { out.message = "invalid cell size"; return out; }

    const auto& positions = scene->getPositions();

    Phantom::PC::DownSampler downSampler;
    for (const auto& q : positions) downSampler.add(q);
    downSampler.execute(p.cellSize);

    const auto sampled = downSampler.getDownSampled();
    if (sampled.empty()) { out.message = "downsample produced empty result"; return out; }

    const float ratio = static_cast<float>(sampled.size()) /
                        static_cast<float>(positions.size());

    auto* result = world.addScene("DownSampleResult");
    for (const auto& q : sampled)
        result->add(q, glm::vec3(ratio, ratio, ratio));
    scene->setVisible(false);

    out.ok             = true;
    out.primarySceneId = result->getId();
    out.message        = "Downsampled " + std::to_string(positions.size()) +
                         " -> " + std::to_string(sampled.size()) + " points";
    return out;
}

ProcessOutcome estimateNormals(World& world, int activeSceneId, const NormalParams& p)
{
    ProcessOutcome out;
    auto* scene = world.findById(activeSceneId);
    if (!scene)        { out.message = "no active scene"; return out; }
    if (p.radius <= 0.f) { out.message = "invalid radius"; return out; }

    const auto& positions = scene->getPositions();

    Phantom::PC::NormalEstimator estimator;
    for (const auto& q : positions) estimator.add(q);
    estimator.estimate(p.radius);
    if (p.orientToViewpoint)
        estimator.orientTowardsViewpoint(p.viewpoint);
    const auto normals = estimator.getNormals();

    auto* result = world.addScene(p.orientToViewpoint ? "OrientedNormalResult"
                                                      : "NormalResult");
    for (size_t i = 0; i < positions.size() && i < normals.size(); ++i) {
        const auto& n = normals[i];
        result->add(positions[i], glm::vec3((n.x + 1.f) * 0.5f,
                                            (n.y + 1.f) * 0.5f,
                                            (n.z + 1.f) * 0.5f));
    }
    result->setNormals(normals);
    scene->setVisible(false);

    out.ok             = true;
    out.primarySceneId = result->getId();
    out.message        = "Estimated normals for " + std::to_string(normals.size()) + " points";
    return out;
}

ProcessOutcome filterDensity(World& world, int activeSceneId, const DensityFilterParams& p)
{
    ProcessOutcome out;
    auto* scene = world.findById(activeSceneId);
    if (!scene)          { out.message = "no active scene"; return out; }
    if (p.radius <= 0.f) { out.message = "invalid radius"; return out; }

    const auto& positions = scene->getPositions();

    Phantom::PC::DensityBasedFilter filter;
    for (const auto& q : positions) filter.add(q);
    filter.execute(p.radius);

    auto* result = world.addScene("DensityFilterResult");
    int kept = 0;
    for (int idx : filter.getInlierIndices()) {
        if (idx >= 0 && static_cast<size_t>(idx) < positions.size()) {
            result->add(positions[idx], glm::vec3(0.6f, 0.85f, 1.0f));
            ++kept;
        }
    }
    scene->setVisible(false);

    out.ok             = true;
    out.primarySceneId = result->getId();
    out.message        = "Kept " + std::to_string(kept) + " / " +
                         std::to_string(positions.size()) + " points";
    return out;
}

ProcessOutcome filterCurvature(World& world, int activeSceneId, const CurvatureFilterParams& p)
{
    ProcessOutcome out;
    auto* scene = world.findById(activeSceneId);
    if (!scene)          { out.message = "no active scene"; return out; }
    if (p.radius <= 0.f) { out.message = "invalid radius"; return out; }

    const auto& positions = scene->getPositions();

    Phantom::PC::CurvatureEstimator estimator;
    for (const auto& q : positions) estimator.add(q);
    estimator.estimate(p.radius);
    const auto curvatures = estimator.getCurvatures();

    const double maxCurv = curvatures.empty() ? 1.0
        : *std::max_element(curvatures.begin(), curvatures.end());
    const double norm = (maxCurv > 0.0) ? maxCurv : 1.0;

    auto* result = world.addScene("CurvatureFilterResult");
    int kept = 0;
    for (size_t i = 0; i < positions.size() && i < curvatures.size(); ++i) {
        if (curvatures[i] <= static_cast<double>(p.threshold)) {
            const float v = static_cast<float>(curvatures[i] / norm);
            result->add(positions[i], glm::vec3(v, v, v));
            ++kept;
        }
    }
    scene->setVisible(false);

    out.ok             = true;
    out.primarySceneId = result->getId();
    out.message        = "Kept " + std::to_string(kept) + " / " +
                         std::to_string(positions.size()) + " points";
    return out;
}

// --- RANSAC ---------------------------------------------------------------

RansacPlaneResult detectPlane(World& world, int activeSceneId, const RansacParams& p)
{
    RansacPlaneResult r;
    Phantom::PC::RansacPlaneDetector::PlaneModel model;
    r.outcome = runRansac<Phantom::PC::RansacPlaneDetector>(
        world, activeSceneId, p, "Plane", glm::vec3(0.2f, 0.9f, 0.3f), model);
    if (!r.outcome.ok) return r;

    if (p.buildOverlayMesh) {
        auto* scene = world.findById(activeSceneId);
        if (scene) buildPlaneMesh(scene->getPositions(), model, world);
    }
    r.normal      = glm::normalize(glm::vec3(model.normal));
    r.offset      = model.offset;
    r.inlierCount = static_cast<int>(model.inliers.size());
    r.outcome.message = "Plane: " + std::to_string(r.inlierCount) + " inliers";
    return r;
}

RansacCylinderResult detectCylinder(World& world, int activeSceneId, const RansacParams& p)
{
    RansacCylinderResult r;
    Phantom::PC::RansacCylinderDetector::CylinderModel model;
    r.outcome = runRansac<Phantom::PC::RansacCylinderDetector>(
        world, activeSceneId, p, "Cylinder", glm::vec3(0.2f, 0.7f, 1.0f), model);
    if (!r.outcome.ok) return r;

    if (p.buildOverlayMesh) {
        auto* scene = world.findById(activeSceneId);
        if (scene) buildCylinderMesh(scene->getPositions(), model, world);
    }
    r.axis        = glm::normalize(glm::vec3(model.direction));
    r.radius      = model.radius;
    r.inlierCount = static_cast<int>(model.inliers.size());
    r.outcome.message = "Cylinder: " + std::to_string(r.inlierCount) + " inliers";
    return r;
}

RansacSphereResult detectSphere(World& world, int activeSceneId, const RansacParams& p)
{
    RansacSphereResult r;
    Phantom::PC::RansacSphereDetector::SphereModel model;
    r.outcome = runRansac<Phantom::PC::RansacSphereDetector>(
        world, activeSceneId, p, "Sphere", glm::vec3(0.9f, 0.6f, 0.1f), model);
    if (!r.outcome.ok) return r;

    r.center      = glm::vec3(model.center);
    r.radius      = model.radius;
    r.inlierCount = static_cast<int>(model.inliers.size());
    r.outcome.message = "Sphere: " + std::to_string(r.inlierCount) + " inliers";
    return r;
}

RansacConeResult detectCone(World& world, int activeSceneId, const RansacParams& p)
{
    RansacConeResult r;
    Phantom::PC::RansacConeDetector::ConeModel model;
    r.outcome = runRansac<Phantom::PC::RansacConeDetector>(
        world, activeSceneId, p, "Cone", glm::vec3(1.0f, 0.7f, 0.2f), model);
    if (!r.outcome.ok) return r;

    r.apex         = glm::vec3(model.apex);
    r.axis         = glm::vec3(model.axis);
    r.halfAngleRad = model.halfAngleRad;
    r.inlierCount  = static_cast<int>(model.inliers.size());
    r.outcome.message = "Cone: " + std::to_string(r.inlierCount) + " inliers";
    return r;
}

// --- Clustering ----------------------------------------------------------

namespace {

// Colour each source point by its per-point cluster id into a new scene, hide
// the source. `clusterId(i)` returns the id for point i.
template <class ClusterIdFn>
ProcessOutcome emitClusterScene(World& world, PointCloudfScene& src,
                                const char* resultName, ClusterIdFn clusterId)
{
    ProcessOutcome out;
    const auto& positions = src.getPositions();
    auto* result = world.addScene(resultName);
    for (size_t i = 0; i < positions.size(); ++i)
        result->add(positions[i], colorFromClusterId(clusterId(i)));
    src.setVisible(false);
    out.ok = true;
    out.primarySceneId = result->getId();
    return out;
}

} // namespace

ClusterResult clusterDbscan(World& world, int activeSceneId, const DbscanParams& p)
{
    ClusterResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)                        { r.outcome.message = "no active scene"; return r; }
    if (p.eps <= 0.f || p.minPts <= 0) { r.outcome.message = "invalid parameters"; return r; }

    auto pts = toPoints(scene->getPositions());
    Phantom::PC::DBSCANClustering clustering;
    clustering.cluster(pts, p.eps, p.minPts);

    int maxId = -1;
    r.outcome = emitClusterScene(world, *scene, "DBSCANResult",
        [&](size_t i) { maxId = std::max(maxId, pts[i].clusterID); return pts[i].clusterID; });
    r.clusterCount = maxId + 1;
    r.outcome.message = "DBSCAN: " + std::to_string(r.clusterCount) + " clusters";
    return r;
}

ClusterResult clusterDistance(World& world, int activeSceneId, const DistanceClusterParams& p)
{
    ClusterResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)          { r.outcome.message = "no active scene"; return r; }
    if (p.radius <= 0.f) { r.outcome.message = "invalid radius"; return r; }

    auto pts = toPoints(scene->getPositions());
    Phantom::PC::DistanceBasedClustering clustering;
    clustering.DistanceBasedRegionGrowing(pts, p.radius);

    int maxId = -1;
    r.outcome = emitClusterScene(world, *scene, "RegionGrowingResult",
        [&](size_t i) { maxId = std::max(maxId, pts[i].clusterID); return pts[i].clusterID; });
    r.clusterCount = maxId + 1;
    r.outcome.message = "Region growing: " + std::to_string(r.clusterCount) + " clusters";
    return r;
}

ClusterResult segmentRegionGrowing(World& world, int activeSceneId, const RegionGrowParams& p)
{
    ClusterResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)              { r.outcome.message = "no active scene"; return r; }
    if (!scene->hasNormals()){ r.outcome.message = "scene has no normals"; return r; }
    if (p.curvatureRadius <= 0.f) { r.outcome.message = "invalid radius"; return r; }

    const auto& positions = scene->getPositions();
    const auto& normals   = scene->getNormals();

    // Curvature isn't persisted on PointCloudfScene, so recompute it from the
    // same radius used for the neighbourhood search.
    Phantom::PC::CurvatureEstimator curvEstimator;
    for (const auto& q : positions) curvEstimator.add(q);
    curvEstimator.estimate(static_cast<double>(p.curvatureRadius));
    const auto curvatures = curvEstimator.getCurvatures();

    Phantom::PC::RegionGrowing regionGrowing;
    for (size_t i = 0; i < positions.size(); ++i)
        regionGrowing.add(positions[i], normals[i], curvatures[i]);

    Phantom::PC::RegionGrowing::Params params;
    params.kNeighbors             = static_cast<size_t>(std::max(1, p.kNeighbors));
    params.smoothnessThresholdRad = p.smoothnessDeg * 3.14159265f / 180.f;
    params.curvatureThreshold     = static_cast<double>(p.curvatureThreshold);
    params.minClusterSize         = static_cast<size_t>(std::max(1, p.minClusterSize));
    if (!regionGrowing.segment(params)) { r.outcome.message = "region growing failed"; return r; }

    const auto labels = regionGrowing.getLabels();
    r.outcome = emitClusterScene(world, *scene, "RegionGrowingNormalResult",
        [&](size_t i) { return labels[i]; });
    r.clusterCount = static_cast<int>(regionGrowing.getClusterCount());
    r.outcome.message = "Region growing: " + std::to_string(r.clusterCount) + " clusters";
    return r;
}

// --- Ground extraction -------------------------------------------------

GroundResult extractGround(World& world, int activeSceneId, const GroundParams& p)
{
    GroundResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)            { r.outcome.message = "no active scene"; return r; }
    if (p.cellSize <= 0.f) { r.outcome.message = "invalid cell size"; return r; }

    const auto& positions = scene->getPositions();
    Phantom::PC::GroundExtractor extractor;
    for (const auto& q : positions) extractor.add(q);

    Phantom::PC::GroundExtractor::Params params;
    params.cellSize                  = p.cellSize;
    params.slope                     = p.slope;
    params.initialWindowSize         = p.initialWindowSize;
    params.maxWindowSize             = p.maxWindowSize;
    params.windowGrowthFactor        = p.windowGrowthFactor;
    params.initialElevationThreshold = p.initialElevationThreshold;
    params.maxElevationThreshold     = p.maxElevationThreshold;
    params.finalElevationThreshold   = p.finalElevationThreshold;
    if (!extractor.extract(params)) { r.outcome.message = "ground extraction failed"; return r; }

    const auto flags = extractor.getGroundFlags();
    auto* ground    = world.addScene("GroundPoints");
    auto* nonGround = world.addScene("NonGroundPoints");
    for (size_t i = 0; i < positions.size() && i < flags.size(); ++i) {
        if (flags[i]) { ground->add(positions[i], glm::vec3(0.4f, 0.8f, 0.3f)); ++r.groundCount; }
        else          { nonGround->add(positions[i], glm::vec3(0.7f, 0.45f, 0.2f)); ++r.nonGroundCount; }
    }
    scene->setVisible(false);

    r.groundRatio = flags.empty() ? 0.0
        : static_cast<double>(r.groundCount) / static_cast<double>(flags.size());
    r.outcome.ok = true;
    r.outcome.primarySceneId = ground->getId();
    r.outcome.message = std::to_string(r.groundCount) + " ground / " +
                        std::to_string(r.nonGroundCount) + " non-ground points";
    return r;
}

// --- Curvature / FPFH / boundary -------------------------------------

CurvatureResult estimateCurvature(World& world, int activeSceneId, const CurvatureParams& p)
{
    CurvatureResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)          { r.outcome.message = "no active scene"; return r; }
    if (p.radius <= 0.f) { r.outcome.message = "invalid radius"; return r; }

    const auto& positions = scene->getPositions();
    Phantom::PC::CurvatureEstimator estimator;
    for (const auto& q : positions) estimator.add(q);

    if (p.principal) {
        estimator.estimatePrincipal(p.radius);
        const auto pc = estimator.getPrincipalCurvatures();
        double maxAbs = 0.0, sumK1 = 0.0, sumK2 = 0.0;
        for (const auto& c : pc) {
            maxAbs = std::max({ maxAbs, std::abs(c.k1), std::abs(c.k2) });
            sumK1 += c.k1; sumK2 += c.k2;
        }
        const double norm = (maxAbs > 0.0) ? maxAbs : 1.0;
        auto* result = world.addScene("PrincipalCurvatureResult");
        for (size_t i = 0; i < positions.size() && i < pc.size(); ++i) {
            const float v = static_cast<float>(
                std::max(std::abs(pc[i].k1), std::abs(pc[i].k2)) / norm);
            result->add(positions[i], glm::vec3(v, v, v));
        }
        r.meanK1 = pc.empty() ? 0.0 : sumK1 / static_cast<double>(pc.size());
        r.meanK2 = pc.empty() ? 0.0 : sumK2 / static_cast<double>(pc.size());
        r.outcome.primarySceneId = result->getId();
        r.outcome.message = "Principal curvature: meanK1=" + std::to_string(r.meanK1) +
                            " meanK2=" + std::to_string(r.meanK2);
    } else {
        estimator.estimate(p.radius);
        const auto curv = estimator.getCurvatures();
        const double maxCurv = curv.empty() ? 1.0
            : *std::max_element(curv.begin(), curv.end());
        const double norm = (maxCurv > 0.0) ? maxCurv : 1.0;
        auto* result = world.addScene("CurvatureResult");
        for (size_t i = 0; i < positions.size() && i < curv.size(); ++i) {
            const float v = static_cast<float>(curv[i] / norm);
            result->add(positions[i], glm::vec3(v, v, v));
        }
        r.outcome.primarySceneId = result->getId();
        r.outcome.message = "Estimated scalar curvature for " +
                            std::to_string(curv.size()) + " points";
    }
    scene->setVisible(false);
    r.outcome.ok = true;
    return r;
}

FpfhResult estimateFpfh(World& world, int activeSceneId, const FpfhParams& p)
{
    FpfhResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)               { r.outcome.message = "no active scene"; return r; }
    if (!scene->hasNormals()) { r.outcome.message = "scene has no normals"; return r; }
    if (p.kNeighbors < 1)     { r.outcome.message = "invalid k"; return r; }

    const auto& positions = scene->getPositions();
    const auto& normals   = scene->getNormals();

    Phantom::PC::FPFHEstimator estimator;
    for (size_t i = 0; i < positions.size(); ++i) estimator.add(positions[i], normals[i]);
    if (!estimator.estimate(static_cast<size_t>(p.kNeighbors))) {
        r.outcome.message = "FPFH estimation failed";
        return r;
    }
    const auto histograms = estimator.getHistograms();

    Phantom::PC::FPFHEstimator::Histogram mean{};
    mean.fill(0.f);
    for (const auto& h : histograms)
        for (size_t b = 0; b < h.size(); ++b)
            mean[b] += h[b] / static_cast<float>(histograms.size());

    std::vector<float> dist(histograms.size(), 0.f);
    float maxDist = 0.f;
    for (size_t i = 0; i < histograms.size(); ++i) {
        float d = 0.f;
        for (size_t b = 0; b < histograms[i].size(); ++b) {
            const float diff = histograms[i][b] - mean[b];
            d += diff * diff;
        }
        dist[i] = std::sqrt(d);
        maxDist = std::max(maxDist, dist[i]);
    }
    const float norm = (maxDist > 0.f) ? maxDist : 1.f;

    auto* result = world.addScene("FPFHResult");
    for (size_t i = 0; i < positions.size(); ++i) {
        const float v = dist[i] / norm;
        result->add(positions[i], glm::vec3(v, 0.3f, 1.0f - v));
    }
    scene->setVisible(false);

    r.outcome.ok = true;
    r.outcome.primarySceneId = result->getId();
    r.descriptorCount = static_cast<int>(histograms.size());
    r.histogramSize   = static_cast<int>(Phantom::PC::FPFHEstimator::HistogramSize);
    r.outcome.message = std::to_string(r.descriptorCount) + " descriptors (" +
                        std::to_string(r.histogramSize) + "-dim)";
    return r;
}

BoundaryResult detectBoundary(World& world, int activeSceneId, const BoundaryParams& p)
{
    BoundaryResult r;
    auto* scene = world.findById(activeSceneId);
    if (!scene)               { r.outcome.message = "no active scene"; return r; }
    if (!scene->hasNormals()) { r.outcome.message = "scene has no normals"; return r; }
    if (p.radius <= 0.f)      { r.outcome.message = "invalid radius"; return r; }

    const auto& positions = scene->getPositions();
    const auto& normals   = scene->getNormals();

    Phantom::PC::BoundaryDetector detector;
    for (size_t i = 0; i < positions.size(); ++i) detector.add(positions[i], normals[i]);
    detector.estimate(static_cast<double>(p.radius),
                      p.angleThresholdDeg * 3.14159265f / 180.f);
    const auto flags = detector.getBoundaryFlags();

    auto* result = world.addScene("BoundaryResult");
    for (size_t i = 0; i < positions.size() && i < flags.size(); ++i) {
        if (flags[i]) { result->add(positions[i], glm::vec3(0.9f, 0.15f, 0.15f)); ++r.boundaryCount; }
        else          { result->add(positions[i], glm::vec3(0.2f, 0.4f, 0.9f)); }
    }
    scene->setVisible(false);

    r.outcome.ok = true;
    r.outcome.primarySceneId = result->getId();
    r.outcome.message = std::to_string(r.boundaryCount) + " boundary points";
    return r;
}

} // namespace ops
} // namespace VPC
