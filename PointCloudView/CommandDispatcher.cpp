#include "CommandDispatcher.h"

#include "World.h"
#include "PointCloudfScene.h"
#include "PointCloudFileLoader.h"
#include "PointCloudOps.h"

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
#include "ICPRegistration.h"
#include "GlobalRegistration.h"
#include "MLSSurface.h"
#include "ConvexHull2D.h"
#include "ConcaveHull2D.h"

#include "CGLib/Math/Matrix3d.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// ---- IScenarioDispatcher ------------------------------------------------

void CommandDispatcher::dispatch(const std::string& command) {
    std::lock_guard<std::mutex> lk(mutex_);
    inputQueue_.push(command);
}

std::vector<std::string> CommandDispatcher::collectResponses() {
    std::vector<std::string> out;
    std::lock_guard<std::mutex> lk(mutex_);
    while (!outputQueue_.empty()) {
        out.push_back(std::move(outputQueue_.front()));
        outputQueue_.pop();
    }
    return out;
}

// ---- processQueue (render thread) ---------------------------------------

void CommandDispatcher::processQueue() {
    std::queue<std::string> local;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        std::swap(local, inputQueue_);
    }
    while (!local.empty()) {
        std::string resp = route(local.front());
        local.pop();
        std::lock_guard<std::mutex> lk(mutex_);
        outputQueue_.push(std::move(resp));
    }
}

// ---- helpers ------------------------------------------------------------

namespace {

static std::string cmdName(const std::string& cmd) {
    auto pos = cmd.find(':');
    return (pos == std::string::npos) ? cmd : cmd.substr(0, pos);
}

static std::string cmdArg(const std::string& cmd) {
    auto pos = cmd.find(':');
    return (pos == std::string::npos) ? "" : cmd.substr(pos + 1);
}

static std::vector<std::string> splitComma(const std::string& s) {
    std::vector<std::string> parts;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) parts.push_back(tok);
    return parts;
}

static glm::vec3 hsvToRgb(float h, float s, float v) {
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

static glm::vec3 colorFromCluster(int id) {
    if (id < 0) return { 0.5f, 0.5f, 0.5f };
    const float hue = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f);
    return hsvToRgb(hue, 0.85f, 1.0f);
}

static float toFloat(const std::string& s, float def = 0.f) {
    float out = def;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return (ec == std::errc{}) ? out : def;
}

static int toInt(const std::string& s, int def = 0) {
    int out = def;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return (ec == std::errc{}) ? out : def;
}

} // namespace

// ---- route --------------------------------------------------------------

std::string CommandDispatcher::route(const std::string& cmd) {
    if (!world_) return "Error:world not set";

    const std::string name = cmdName(cmd);
    const std::string arg  = cmdArg(cmd);

    // --- Scene management ---

    if (name == "Clear") {
        world_->clear();
        activeId() = -1;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "OK";
    }

    if (name == "GetSceneCount") {
        return "Count:" + std::to_string(world_->getScenes().size());
    }

    if (name == "GetLastSceneId") {
        const auto& scenes = world_->getScenes();
        return scenes.empty() ? "Id:-1" : "Id:" + std::to_string(scenes.back()->getId());
    }

    if (name == "GetScenePointCount") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        return "Count:" + std::to_string(scene->getSize());
    }

    if (name == "GetSceneHasNormals") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        return scene->hasNormals() ? "Yes" : "No";
    }

    if (name == "GetSceneNormalCount") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        return "Count:" + std::to_string(scene->getNormals().size());
    }

    if (name == "GetLastMetric") {
        auto it = lastMetrics_.find(arg);
        return (it == lastMetrics_.end()) ? "Error:no such metric" : it->second;
    }

    if (name == "SetActiveScene") {
        int id = toInt(arg, -1);
        auto* scene = world_->findById(id);
        if (!scene) return "Error:scene not found";
        activeId() = id;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "OK";
    }

    // --- Generate ---

    if (name == "GenerateSphere") {
        int count = toInt(arg, 1000);
        if (count <= 0) return "Error:invalid count";

        std::mt19937 rng(42); // fixed seed for deterministic results
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto* result = world_->addScene("Sphere");
        for (int i = 0; i < count; ++i) {
            const float u     = dist(rng);
            const float v     = dist(rng);
            const float theta = 2.f * 3.14159265f * u;
            const float phi   = std::acosf(1.f - 2.f * v);
            const glm::vec3 pos{
                std::sinf(phi) * std::cosf(theta),
                std::sinf(phi) * std::sinf(theta),
                std::cosf(phi)
            };
            result->add(pos, glm::vec3(0.6f, 0.8f, 1.0f));
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GenerateCylinder") {
        int count = toInt(arg, 500);
        if (count <= 0) return "Error:invalid count";

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto* result = world_->addScene("Cylinder");
        for (int i = 0; i < count; ++i) {
            const float u     = dist(rng);
            const float v     = dist(rng);
            const float theta = 2.f * 3.14159265f * u;
            const glm::vec3 pos{ std::cosf(theta), v, std::sinf(theta) };
            result->add(pos, glm::vec3(0.8f, 0.6f, 1.0f));
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GenerateRect") {
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:GenerateRect expects u,v args";
        const int uCount = toInt(parts[0], 10);
        const int vCount = toInt(parts[1], 10);
        if (uCount <= 0 || vCount <= 0) return "Error:invalid dimensions";

        auto* result = world_->addScene("Rect");
        for (int i = 0; i < uCount; ++i) {
            const float ut = static_cast<float>(i) / static_cast<float>(uCount - 1);
            for (int j = 0; j < vCount; ++j) {
                const float vt = static_cast<float>(j) / static_cast<float>(vCount - 1);
                result->add(glm::vec3(ut, 0.f, vt), glm::vec3(ut, vt, 1.f));
            }
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GenerateCone") {
        // Cone with apex at the origin, opening toward +Y, base radius 1 at height 1.
        // Kept as a local synthesizer (like GenerateSphere/GenerateCylinder above) rather than
        // a Phantom::PC::PrimitiveGenerator addition -- see PLAN §5.3.
        int count = toInt(arg, 800);
        if (count <= 0) return "Error:invalid count";

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto* result = world_->addScene("Cone");
        for (int i = 0; i < count; ++i) {
            const float u = dist(rng);
            const float v = dist(rng);
            const float theta = 2.f * 3.14159265f * u;
            const float h = v; // height along +Y, in [0,1]
            const glm::vec3 pos{ h * std::cosf(theta), h, h * std::sinf(theta) };
            result->add(pos, glm::vec3(1.0f, 0.8f, 0.5f));
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GenerateGroundScene") {
        // Flat ground plane plus a raised block, for GroundExtractor testing.
        // GroundExtractor.cpp buckets points by (x,y) into its 2D grid and treats z as
        // elevation (the usual LiDAR/terrain convention) -- unlike this file's other
        // synthetic generators (GenerateSphere/Cylinder/Rect), which put "up" on Y. Keep
        // this one Z-up so points actually land in the ground/non-ground grid as intended.
        int count = toInt(arg, 2000);
        if (count <= 0) return "Error:invalid count";

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(-5.f, 5.f);
        std::uniform_real_distribution<float> noise(-0.02f, 0.02f);

        auto* result = world_->addScene("GroundScene");
        const int groundCount = static_cast<int>(count * 0.8);
        for (int i = 0; i < groundCount; ++i) {
            const float x = dist(rng);
            const float y = dist(rng);
            result->add(glm::vec3(x, y, noise(rng)), glm::vec3(0.6f, 0.8f, 0.4f));
        }

        const int blockCount = count - groundCount;
        std::uniform_real_distribution<float> blockXY(-1.5f, 1.5f);
        std::uniform_real_distribution<float> blockZ(0.f, 2.0f);
        for (int i = 0; i < blockCount; ++i) {
            const float x = blockXY(rng);
            const float y = blockXY(rng);
            const float z = blockZ(rng);
            result->add(glm::vec3(x, y, z), glm::vec3(0.9f, 0.5f, 0.3f));
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "DuplicateSceneTransformed") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 4) return "Error:DuplicateSceneTransformed expects tx,ty,tz,rotDegY";
        const float tx = toFloat(parts[0], 0.f);
        const float ty = toFloat(parts[1], 0.f);
        const float tz = toFloat(parts[2], 0.f);
        const float rotDegY = toFloat(parts[3], 0.f);
        const float rotRadY = rotDegY * 3.14159265f / 180.f;

        const auto rot = Phantom::Math::rotationMatrixY<float>(rotRadY);
        const glm::vec3 translation(tx, ty, tz);

        auto* result = world_->addScene("DuplicateTransformed");
        for (const auto& p : scene->getPositions()) {
            result->add(rot * p + translation, glm::vec3(0.9f, 0.9f, 0.3f));
        }

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- File I/O ---

    if (name == "LoadPly") {
        if (arg.empty()) return "Error:missing path";
        Phantom::PC::PointCloudColoredData data;
        std::string errorMsg;
        if (!Phantom::PC::loadPointCloud(arg, data, errorMsg))
            return "Error:" + errorMsg;

        const auto sceneName = arg.substr(arg.find_last_of("/\\") + 1);
        auto* scene = world_->addScene(sceneName);
        for (size_t i = 0; i < data.size(); ++i)
            scene->add(data.positions[i], data.colors[i]);

        activeId() = scene->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Processing (operate on active scene) ---

    if (name == "DownSample") {
        // Shared with DownSamplerView (GUI) so both paths match exactly
        // (PLAN Phase 4). The old inline error strings are preserved by
        // prefixing "Error:" to the outcome message.
        VPC::ops::DownSampleParams p;
        p.cellSize = toFloat(arg, 0.05f);
        const auto outcome = VPC::ops::downSample(*world_, activeId(), p);
        if (!outcome.ok) return "Error:" + outcome.message;

        activeId() = outcome.resultSceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimateNormals") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float radius = toFloat(arg, 0.1f);
        if (radius <= 0.f) return "Error:invalid radius";

        Phantom::PC::NormalEstimator estimator;
        const auto& positions = scene->getPositions();
        for (const auto& p : positions) estimator.add(p);
        estimator.estimate(radius);
        const auto normals = estimator.getNormals();

        auto* result = world_->addScene("NormalResult");
        for (size_t i = 0; i < positions.size() && i < normals.size(); ++i) {
            const auto& n = normals[i];
            result->add(positions[i],
                        glm::vec3((n.x + 1.f) * 0.5f,
                                  (n.y + 1.f) * 0.5f,
                                  (n.z + 1.f) * 0.5f));
        }
        result->setNormals(normals);
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "FilterDensity") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float radius = toFloat(arg, 0.1f);
        if (radius <= 0.f) return "Error:invalid radius";

        Phantom::PC::DensityBasedFilter filter;
        const auto& positions = scene->getPositions();
        for (const auto& p : positions) filter.add(p);
        filter.execute(radius);

        auto* result = world_->addScene("DensityFilterResult");
        for (int idx : filter.getInlierIndices()) {
            if (idx >= 0 && static_cast<size_t>(idx) < positions.size())
                result->add(positions[idx], glm::vec3(0.6f, 0.85f, 1.0f));
        }
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "FilterCurvature") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:FilterCurvature expects radius,threshold";
        const float radius    = toFloat(parts[0], 0.1f);
        const float threshold = toFloat(parts[1], 0.01f);
        if (radius <= 0.f) return "Error:invalid radius";

        Phantom::PC::CurvatureEstimator estimator;
        const auto& positions = scene->getPositions();
        for (const auto& p : positions) estimator.add(p);
        estimator.estimate(radius);
        const auto curvatures = estimator.getCurvatures();

        const double maxCurv = curvatures.empty() ? 1.0
            : *std::max_element(curvatures.begin(), curvatures.end());
        const double norm = (maxCurv > 0.0) ? maxCurv : 1.0;

        auto* result = world_->addScene("CurvatureFilterResult");
        for (size_t i = 0; i < positions.size() && i < curvatures.size(); ++i) {
            if (curvatures[i] <= static_cast<double>(threshold)) {
                const float v = static_cast<float>(curvatures[i] / norm);
                result->add(positions[i], glm::vec3(v, v, v));
            }
        }
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacPlane") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float threshold = toFloat(arg, 0.05f);

        const auto& positions = scene->getPositions();
        Phantom::PC::RansacPlaneDetector detector;
        Phantom::PC::RansacPlaneDetector::PlaneModel model;
        if (!detector.detect(positions, model, 200, threshold, 50))
            return "Error:plane detection failed";

        std::vector<bool> isInlier(positions.size(), false);
        auto* inliers = world_->addScene("PlaneInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.2f, 0.9f, 0.3f));
            }
        }
        auto* outliers = world_->addScene("PlaneOutliers");
        for (size_t i = 0; i < positions.size(); ++i)
            if (!isInlier[i]) outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
        scene->setVisible(false);

        activeId() = inliers->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacCylinder") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float threshold = toFloat(arg, 0.05f);

        const auto& positions = scene->getPositions();
        Phantom::PC::RansacCylinderDetector detector;
        Phantom::PC::RansacCylinderDetector::CylinderModel model;
        if (!detector.detect(positions, model, 200, threshold, 50))
            return "Error:cylinder detection failed";

        std::vector<bool> isInlier(positions.size(), false);
        auto* inliers = world_->addScene("CylinderInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.2f, 0.7f, 1.0f));
            }
        }
        auto* outliers = world_->addScene("CylinderOutliers");
        for (size_t i = 0; i < positions.size(); ++i)
            if (!isInlier[i]) outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
        scene->setVisible(false);

        activeId() = inliers->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ClusterDbscan") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ClusterDbscan expects eps,minPts";
        const float eps    = toFloat(parts[0], 0.03f);
        const int   minPts = toInt(parts[1], 8);
        if (eps <= 0.f || minPts <= 0) return "Error:invalid parameters";

        const auto& positions = scene->getPositions();
        std::vector<Phantom::PC::Point> pts;
        pts.reserve(positions.size());
        for (const auto& p : positions)
            pts.emplace_back(static_cast<double>(p.x),
                             static_cast<double>(p.y),
                             static_cast<double>(p.z));

        Phantom::PC::DBSCANClustering clustering;
        clustering.cluster(pts, eps, minPts);

        auto* result = world_->addScene("DBSCANResult");
        for (const auto& p : positions) result->add(p, glm::vec3(0.6f, 0.8f, 1.0f));
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ClusterRegionGrowing") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float radius = toFloat(arg, 0.03f);
        if (radius <= 0.f) return "Error:invalid radius";

        const auto& positions = scene->getPositions();
        std::vector<Phantom::PC::Point> pts;
        pts.reserve(positions.size());
        for (const auto& p : positions)
            pts.emplace_back(static_cast<double>(p.x),
                             static_cast<double>(p.y),
                             static_cast<double>(p.z));

        Phantom::PC::DistanceBasedClustering clustering;
        clustering.DistanceBasedRegionGrowing(pts, radius);

        auto* result = world_->addScene("RegionGrowingResult");
        for (const auto& p : positions) result->add(p, glm::vec3(0.8f, 0.7f, 1.0f));
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase B: normal orientation / principal curvature / FPFH / boundary ---

    if (name == "OrientNormals") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 4) return "Error:OrientNormals expects radius,vx,vy,vz";
        const float radius = toFloat(parts[0], 0.1f);
        const glm::vec3 viewpoint(toFloat(parts[1], 0.f), toFloat(parts[2], 0.f), toFloat(parts[3], 0.f));
        if (radius <= 0.f) return "Error:invalid radius";

        const auto& positions = scene->getPositions();
        Phantom::PC::NormalEstimator estimator;
        for (const auto& p : positions) estimator.add(p);
        estimator.estimate(radius);
        estimator.orientTowardsViewpoint(viewpoint);
        const auto normals = estimator.getNormals();

        auto* result = world_->addScene("OrientedNormalResult");
        for (size_t i = 0; i < positions.size() && i < normals.size(); ++i) {
            const auto& n = normals[i];
            result->add(positions[i],
                        glm::vec3((n.x + 1.f) * 0.5f, (n.y + 1.f) * 0.5f, (n.z + 1.f) * 0.5f));
        }
        result->setNormals(normals);
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimatePrincipalCurvature") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float radius = toFloat(arg, 0.1f);
        if (radius <= 0.f) return "Error:invalid radius";

        const auto& positions = scene->getPositions();
        Phantom::PC::CurvatureEstimator estimator;
        for (const auto& p : positions) estimator.add(p);
        estimator.estimatePrincipal(radius);
        const auto pc = estimator.getPrincipalCurvatures();

        double maxAbs = 0.0, sumK1 = 0.0, sumK2 = 0.0;
        for (const auto& c : pc) {
            maxAbs = std::max({ maxAbs, std::abs(c.k1), std::abs(c.k2) });
            sumK1 += c.k1; sumK2 += c.k2;
        }
        const double norm = (maxAbs > 0.0) ? maxAbs : 1.0;

        auto* result = world_->addScene("PrincipalCurvatureResult");
        for (size_t i = 0; i < positions.size() && i < pc.size(); ++i) {
            const float v = static_cast<float>(std::max(std::abs(pc[i].k1), std::abs(pc[i].k2)) / norm);
            result->add(positions[i], glm::vec3(v, v, v));
        }
        scene->setVisible(false);

        lastMetrics_["meanK1"] = std::to_string(pc.empty() ? 0.0 : sumK1 / static_cast<double>(pc.size()));
        lastMetrics_["meanK2"] = std::to_string(pc.empty() ? 0.0 : sumK2 / static_cast<double>(pc.size()));

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimateFPFH") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        if (!scene->hasNormals()) return "Error:scene has no normals";
        const int k = toInt(arg, 20);
        if (k < 1) return "Error:invalid k";

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        Phantom::PC::FPFHEstimator estimator;
        for (size_t i = 0; i < positions.size(); ++i) estimator.add(positions[i], normals[i]);
        if (!estimator.estimate(static_cast<size_t>(k))) return "Error:FPFH estimation failed";
        const auto histograms = estimator.getHistograms();

        Phantom::PC::FPFHEstimator::Histogram mean{};
        mean.fill(0.f);
        for (const auto& h : histograms)
            for (size_t b = 0; b < h.size(); ++b) mean[b] += h[b] / static_cast<float>(histograms.size());

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

        auto* result = world_->addScene("FPFHResult");
        for (size_t i = 0; i < positions.size(); ++i) {
            const float v = dist[i] / norm;
            result->add(positions[i], glm::vec3(v, 0.3f, 1.0f - v));
        }
        scene->setVisible(false);

        lastMetrics_["fpfhDim"] = std::to_string(Phantom::PC::FPFHEstimator::HistogramSize);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "DetectBoundary") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        if (!scene->hasNormals()) return "Error:scene has no normals";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:DetectBoundary expects radius,angleThresholdDeg";
        const float radius       = toFloat(parts[0], 0.1f);
        const float angleThresholdDeg = toFloat(parts[1], 153.0f);
        if (radius <= 0.f) return "Error:invalid radius";
        const float angleThresholdRad = angleThresholdDeg * 3.14159265f / 180.f;

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        Phantom::PC::BoundaryDetector detector;
        for (size_t i = 0; i < positions.size(); ++i) detector.add(positions[i], normals[i]);
        detector.estimate(static_cast<double>(radius), angleThresholdRad);
        const auto flags = detector.getBoundaryFlags();

        auto* result = world_->addScene("BoundaryResult");
        int boundaryCount = 0;
        for (size_t i = 0; i < positions.size() && i < flags.size(); ++i) {
            if (flags[i]) {
                result->add(positions[i], glm::vec3(0.9f, 0.15f, 0.15f));
                ++boundaryCount;
            } else {
                result->add(positions[i], glm::vec3(0.2f, 0.4f, 0.9f));
            }
        }
        scene->setVisible(false);

        lastMetrics_["boundaryCount"] = std::to_string(boundaryCount);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase C: sphere/cone RANSAC, normal-based region growing, ground extraction ---

    if (name == "RansacSphere") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float threshold = toFloat(arg, 0.02f);

        const auto& positions = scene->getPositions();
        Phantom::PC::RansacSphereDetector detector;
        Phantom::PC::RansacSphereDetector::SphereModel model;
        if (!detector.detect(positions, model, 200, threshold, 50))
            return "Error:sphere detection failed";

        std::vector<bool> isInlier(positions.size(), false);
        auto* inliers = world_->addScene("SphereInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(0.9f, 0.6f, 0.1f));
            }
        }
        auto* outliers = world_->addScene("SphereOutliers");
        for (size_t i = 0; i < positions.size(); ++i)
            if (!isInlier[i]) outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
        scene->setVisible(false);

        lastMetrics_["sphereRadius"] = std::to_string(model.radius);

        activeId() = inliers->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacCone") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float threshold = toFloat(arg, 0.02f);

        const auto& positions = scene->getPositions();
        Phantom::PC::RansacConeDetector detector;
        Phantom::PC::RansacConeDetector::ConeModel model;
        if (!detector.detect(positions, model, 200, threshold, 50))
            return "Error:cone detection failed";

        std::vector<bool> isInlier(positions.size(), false);
        auto* inliers = world_->addScene("ConeInliers");
        for (const auto idx : model.inliers) {
            if (idx < positions.size()) {
                isInlier[idx] = true;
                inliers->add(positions[idx], glm::vec3(1.0f, 0.7f, 0.2f));
            }
        }
        auto* outliers = world_->addScene("ConeOutliers");
        for (size_t i = 0; i < positions.size(); ++i)
            if (!isInlier[i]) outliers->add(positions[i], glm::vec3(0.8f, 0.2f, 0.2f));
        scene->setVisible(false);

        lastMetrics_["coneHalfAngleRad"] = std::to_string(model.halfAngleRad);

        activeId() = inliers->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "SegmentRegionGrowing") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        if (!scene->hasNormals()) return "Error:scene has no normals";
        const auto parts = splitComma(arg);
        if (parts.size() < 4) return "Error:SegmentRegionGrowing expects radius,kNeighbors,smoothnessDeg,curvatureThreshold";
        const float radius         = toFloat(parts[0], 0.1f);
        const int   kNeighbors     = toInt(parts[1], 30);
        const float smoothnessDeg  = toFloat(parts[2], 5.0f);
        const float curvatureThreshold = toFloat(parts[3], 1.0f);
        if (radius <= 0.f) return "Error:invalid radius";

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        // Curvature isn't persisted on PointCloudfScene, so it's recomputed here from the same
        // radius (see PLAN §4 Phase C: RegionGrowingView note on self-contained operation).
        Phantom::PC::CurvatureEstimator curvEstimator;
        for (const auto& p : positions) curvEstimator.add(p);
        curvEstimator.estimate(static_cast<double>(radius));
        const auto curvatures = curvEstimator.getCurvatures();

        Phantom::PC::RegionGrowing regionGrowing;
        for (size_t i = 0; i < positions.size(); ++i)
            regionGrowing.add(positions[i], normals[i], curvatures[i]);

        Phantom::PC::RegionGrowing::Params params;
        params.kNeighbors = static_cast<size_t>(std::max(1, kNeighbors));
        params.smoothnessThresholdRad = smoothnessDeg * 3.14159265f / 180.f;
        params.curvatureThreshold = static_cast<double>(curvatureThreshold);
        if (!regionGrowing.segment(params)) return "Error:region growing failed";

        const auto labels = regionGrowing.getLabels();
        auto* result = world_->addScene("RegionGrowingNormalResult");
        for (size_t i = 0; i < positions.size(); ++i)
            result->add(positions[i], colorFromCluster(labels[i]));
        scene->setVisible(false);

        lastMetrics_["clusterCount"] = std::to_string(regionGrowing.getClusterCount());

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ExtractGround") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ExtractGround expects cellSize,slope";
        const float cellSize = toFloat(parts[0], 1.0f);
        const float slope    = toFloat(parts[1], 0.3f);
        if (cellSize <= 0.f) return "Error:invalid cell size";

        const auto& positions = scene->getPositions();
        Phantom::PC::GroundExtractor extractor;
        for (const auto& p : positions) extractor.add(p);

        Phantom::PC::GroundExtractor::Params params;
        params.cellSize = cellSize;
        params.slope    = slope;
        if (!extractor.extract(params)) return "Error:ground extraction failed";
        const auto groundFlags = extractor.getGroundFlags();

        auto* ground = world_->addScene("GroundPoints");
        auto* nonGround = world_->addScene("NonGroundPoints");
        int groundCount = 0;
        for (size_t i = 0; i < positions.size() && i < groundFlags.size(); ++i) {
            if (groundFlags[i]) { ground->add(positions[i], glm::vec3(0.4f, 0.8f, 0.3f)); ++groundCount; }
            else                { nonGround->add(positions[i], glm::vec3(0.7f, 0.45f, 0.2f)); }
        }
        scene->setVisible(false);

        lastMetrics_["groundRatio"] = std::to_string(
            groundFlags.empty() ? 0.0 : static_cast<double>(groundCount) / static_cast<double>(groundFlags.size()));

        activeId() = ground->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase D: MLS surface smoothing/upsampling, 2D convex/concave hull ---

    if (name == "MLSSmooth") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const float radius = toFloat(arg, 0.1f);
        if (radius <= 0.f) return "Error:invalid radius";

        const auto& positions = scene->getPositions();
        Phantom::PC::MLSSurface mls;
        for (const auto& p : positions) mls.add(p);
        mls.smooth(static_cast<double>(radius));
        const auto smoothed = mls.getSmoothedPoints();

        auto* result = world_->addScene("MLSSmoothed");
        for (const auto& p : smoothed) result->add(p, glm::vec3(0.5f, 0.85f, 0.9f));
        scene->setVisible(false);

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "MLSUpsample") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 3) return "Error:MLSUpsample expects radius,upsampleRadius,stepSize";
        const float radius         = toFloat(parts[0], 0.1f);
        const float upsampleRadius = toFloat(parts[1], 0.05f);
        const float stepSize       = toFloat(parts[2], 0.01f);
        if (radius <= 0.f || stepSize <= 0.f) return "Error:invalid parameters";

        const auto& positions = scene->getPositions();
        Phantom::PC::MLSSurface mls;
        for (const auto& p : positions) mls.add(p);
        const auto upsampled = mls.upsample(static_cast<double>(radius), upsampleRadius, stepSize);
        if (upsampled.empty()) return "Error:upsample produced no points";

        auto* result = world_->addScene("MLSUpsampled");
        for (const auto& p : upsampled) result->add(p, glm::vec3(0.9f, 0.7f, 0.9f));

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ConvexHull2D") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";

        Phantom::PC::ConvexHull2D hull;
        for (const auto& p : scene->getPositions()) hull.add(p);
        if (!hull.compute()) return "Error:convex hull failed";
        const auto verts = hull.getHullPoints();

        auto* result = world_->addScene("ConvexHullVertices");
        for (const auto& v : verts) result->add(v, glm::vec3(0.3f, 0.9f, 0.6f));

        world_->clearPolygons();
        VPC::PolygonMesh mesh;
        mesh.name = "ConvexHullPolygon";
        for (const auto& v : verts) {
            mesh.positions.insert(mesh.positions.end(), { v.x, v.y, v.z });
            mesh.colors.insert(mesh.colors.end(), { 0.3f, 0.9f, 0.6f, 0.5f });
        }
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            mesh.indices.push_back(0);
            mesh.indices.push_back(static_cast<uint32_t>(i));
            mesh.indices.push_back(static_cast<uint32_t>(i + 1));
        }
        world_->addPolygon(std::move(mesh));

        lastMetrics_["hullArea"] = std::to_string(hull.getArea());

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ConcaveHull2D") {
        auto* scene = world_->findById(activeId());
        if (!scene) return "Error:no active scene";
        const auto parts = splitComma(arg);
        const size_t k    = parts.empty() ? 3 : static_cast<size_t>(std::max(3, toInt(parts[0], 3)));
        const size_t maxK = parts.size() < 2 ? 0 : static_cast<size_t>(std::max(0, toInt(parts[1], 0)));

        Phantom::PC::ConcaveHull2D hull;
        for (const auto& p : scene->getPositions()) hull.add(p);
        if (!hull.compute(k, maxK)) return "Error:concave hull failed";
        const auto verts = hull.getHullPoints();

        auto* result = world_->addScene("ConcaveHullVertices");
        for (const auto& v : verts) result->add(v, glm::vec3(0.9f, 0.6f, 0.3f));

        world_->clearPolygons();
        VPC::PolygonMesh mesh;
        mesh.name = "ConcaveHullPolygon";
        for (const auto& v : verts) {
            mesh.positions.insert(mesh.positions.end(), { v.x, v.y, v.z });
            mesh.colors.insert(mesh.colors.end(), { 0.9f, 0.6f, 0.3f, 0.5f });
        }
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            mesh.indices.push_back(0);
            mesh.indices.push_back(static_cast<uint32_t>(i));
            mesh.indices.push_back(static_cast<uint32_t>(i + 1));
        }
        world_->addPolygon(std::move(mesh));

        lastMetrics_["hullArea"] = std::to_string(hull.getArea());

        activeId() = result->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase A: ICP / global (FPFH+RANSAC) registration ---

    if (name == "ICPAlign") {
        auto* source = world_->findById(activeId());
        if (!source) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ICPAlign expects targetId,maxIterations";
        const int targetId     = toInt(parts[0], -1);
        const int maxIterations = toInt(parts[1], 50);
        auto* target = world_->findById(targetId);
        if (!target) return "Error:target scene not found";

        Phantom::PC::ICPRegistration icp;
        Phantom::PC::ICPRegistration::Result result;
        if (!icp.align(source->getPositions(), target->getPositions(), result, maxIterations))
            return "Error:ICP alignment failed";

        auto* aligned = world_->addScene("ICPAligned");
        for (const auto& p : source->getPositions())
            aligned->add(Phantom::PC::ICPRegistration::transformPoint(result, p), glm::vec3(0.3f, 1.0f, 0.5f));
        source->setVisible(false);

        lastMetrics_["fitness"]    = std::to_string(result.fitness);
        lastMetrics_["iterations"] = std::to_string(result.iterations);
        lastMetrics_["converged"]  = result.converged ? "Yes" : "No";
        lastMetrics_["scale"]      = std::to_string(result.scale);

        activeId() = aligned->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ICPAlignPointToPlane") {
        auto* source = world_->findById(activeId());
        if (!source) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ICPAlignPointToPlane expects targetId,maxIterations";
        const int targetId     = toInt(parts[0], -1);
        const int maxIterations = toInt(parts[1], 50);
        auto* target = world_->findById(targetId);
        if (!target) return "Error:target scene not found";
        if (!target->hasNormals()) return "Error:target has no normals";

        Phantom::PC::ICPRegistration icp;
        Phantom::PC::ICPRegistration::Result result;
        if (!icp.alignPointToPlane(source->getPositions(), target->getPositions(), target->getNormals(),
                                    result, maxIterations))
            return "Error:ICP point-to-plane alignment failed";

        auto* aligned = world_->addScene("ICPAlignedP2Plane");
        for (const auto& p : source->getPositions())
            aligned->add(Phantom::PC::ICPRegistration::transformPoint(result, p), glm::vec3(0.3f, 0.7f, 1.0f));
        source->setVisible(false);

        lastMetrics_["fitness"]    = std::to_string(result.fitness);
        lastMetrics_["iterations"] = std::to_string(result.iterations);
        lastMetrics_["converged"]  = result.converged ? "Yes" : "No";

        activeId() = aligned->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GlobalRegister") {
        auto* source = world_->findById(activeId());
        if (!source) return "Error:no active scene";
        const auto parts = splitComma(arg);
        if (parts.size() < 3) return "Error:GlobalRegister expects targetId,k,iterations";
        const int targetId   = toInt(parts[0], -1);
        const int k          = toInt(parts[1], 20);
        const int iterations = toInt(parts[2], 1000);
        auto* target = world_->findById(targetId);
        if (!target) return "Error:target scene not found";
        if (!source->hasNormals() || !target->hasNormals())
            return "Error:source/target must have normals (run EstimateNormals first)";

        Phantom::PC::FPFHEstimator sourceFpfh, targetFpfh;
        const auto& sp = source->getPositions();
        const auto& sn = source->getNormals();
        for (size_t i = 0; i < sp.size(); ++i) sourceFpfh.add(sp[i], sn[i]);
        const auto& tp = target->getPositions();
        const auto& tn = target->getNormals();
        for (size_t i = 0; i < tp.size(); ++i) targetFpfh.add(tp[i], tn[i]);
        if (!sourceFpfh.estimate(static_cast<size_t>(k)) || !targetFpfh.estimate(static_cast<size_t>(k)))
            return "Error:FPFH estimation failed";

        Phantom::PC::GlobalRegistration globalReg;
        Phantom::PC::GlobalRegistration::Result result;
        if (!globalReg.align(sp, sourceFpfh.getHistograms(), tp, targetFpfh.getHistograms(),
                              result, iterations))
            return "Error:global registration failed";

        auto* aligned = world_->addScene("GlobalRegAligned");
        for (const auto& p : sp) aligned->add(result.rotation * p + result.translation, glm::vec3(1.0f, 0.5f, 0.8f));
        source->setVisible(false);

        lastMetrics_["inlierCount"] = std::to_string(result.inlierCount);
        lastMetrics_["inlierRmse"]  = std::to_string(result.inlierRmse);

        activeId() = aligned->getId();
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    return "Error:unknown command '" + cmd + "'";
}
