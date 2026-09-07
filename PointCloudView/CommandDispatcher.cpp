#include "CommandDispatcher.h"

#include "World.h"
#include "PointCloudfScene.h"
#include "PointCloudFileLoader.h"
#include "PointCloudOps.h"   // every processing command now routes through VPC::ops

#include "CGLib/Math/Matrix3d.h"  // DuplicateSceneTransformed

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

        activeId() = outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimateNormals") {
        VPC::ops::NormalParams p;
        p.radius = toFloat(arg, 0.1f);
        const auto o = VPC::ops::estimateNormals(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "FilterDensity") {
        VPC::ops::DensityFilterParams p;
        p.radius = toFloat(arg, 0.1f);
        const auto o = VPC::ops::filterDensity(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "FilterCurvature") {
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:FilterCurvature expects radius,threshold";
        VPC::ops::CurvatureFilterParams p;
        p.radius    = toFloat(parts[0], 0.1f);
        p.threshold = toFloat(parts[1], 0.01f);
        const auto o = VPC::ops::filterCurvature(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacPlane") {
        VPC::ops::RansacParams p;
        p.threshold = toFloat(arg, 0.05f);
        const auto r = VPC::ops::detectPlane(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacCylinder") {
        VPC::ops::RansacParams p;
        p.threshold = toFloat(arg, 0.05f);
        const auto r = VPC::ops::detectCylinder(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ClusterDbscan") {
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ClusterDbscan expects eps,minPts";
        VPC::ops::DbscanParams p;
        p.eps    = toFloat(parts[0], 0.03f);
        p.minPts = toInt(parts[1], 8);
        const auto r = VPC::ops::clusterDbscan(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ClusterRegionGrowing") {
        VPC::ops::DistanceClusterParams p;
        p.radius = toFloat(arg, 0.03f);
        const auto r = VPC::ops::clusterDistance(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase B: normal orientation / principal curvature / FPFH / boundary ---

    if (name == "OrientNormals") {
        const auto parts = splitComma(arg);
        if (parts.size() < 4) return "Error:OrientNormals expects radius,vx,vy,vz";
        VPC::ops::NormalParams p;
        p.radius            = toFloat(parts[0], 0.1f);
        p.orientToViewpoint = true;
        p.viewpoint = glm::vec3(toFloat(parts[1], 0.f), toFloat(parts[2], 0.f), toFloat(parts[3], 0.f));
        const auto o = VPC::ops::estimateNormals(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimatePrincipalCurvature") {
        VPC::ops::CurvatureParams p;
        p.radius    = toFloat(arg, 0.1f);
        p.principal = true;
        const auto r = VPC::ops::estimateCurvature(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["meanK1"] = std::to_string(r.meanK1);
        lastMetrics_["meanK2"] = std::to_string(r.meanK2);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "EstimateFPFH") {
        VPC::ops::FpfhParams p;
        p.kNeighbors = toInt(arg, 20);
        const auto r = VPC::ops::estimateFpfh(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["fpfhDim"] = std::to_string(r.histogramSize);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "DetectBoundary") {
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:DetectBoundary expects radius,angleThresholdDeg";
        VPC::ops::BoundaryParams p;
        p.radius            = toFloat(parts[0], 0.1f);
        p.angleThresholdDeg = toFloat(parts[1], 153.0f);
        const auto r = VPC::ops::detectBoundary(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["boundaryCount"] = std::to_string(r.boundaryCount);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase C: sphere/cone RANSAC, normal-based region growing, ground extraction ---

    if (name == "RansacSphere") {
        VPC::ops::RansacParams p;
        p.threshold = toFloat(arg, 0.02f);
        const auto r = VPC::ops::detectSphere(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["sphereRadius"] = std::to_string(r.radius);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "RansacCone") {
        VPC::ops::RansacParams p;
        p.threshold = toFloat(arg, 0.02f);
        const auto r = VPC::ops::detectCone(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["coneHalfAngleRad"] = std::to_string(r.halfAngleRad);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "SegmentRegionGrowing") {
        const auto parts = splitComma(arg);
        if (parts.size() < 4) return "Error:SegmentRegionGrowing expects radius,kNeighbors,smoothnessDeg,curvatureThreshold";
        VPC::ops::RegionGrowParams p;
        p.curvatureRadius    = toFloat(parts[0], 0.1f);
        p.kNeighbors         = toInt(parts[1], 30);
        p.smoothnessDeg      = toFloat(parts[2], 5.0f);
        p.curvatureThreshold = toFloat(parts[3], 1.0f);
        const auto r = VPC::ops::segmentRegionGrowing(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["clusterCount"] = std::to_string(r.clusterCount);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ExtractGround") {
        const auto parts = splitComma(arg);
        if (parts.size() < 2) return "Error:ExtractGround expects cellSize,slope";
        VPC::ops::GroundParams p;
        p.cellSize = toFloat(parts[0], 1.0f);
        p.slope    = toFloat(parts[1], 0.3f);
        const auto r = VPC::ops::extractGround(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["groundRatio"] = std::to_string(r.groundRatio);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase D: MLS surface smoothing/upsampling, 2D convex/concave hull ---

    if (name == "MLSSmooth") {
        VPC::ops::MlsSmoothParams p;
        p.radius = toFloat(arg, 0.1f);
        const auto o = VPC::ops::mlsSmooth(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "MLSUpsample") {
        const auto parts = splitComma(arg);
        if (parts.size() < 3) return "Error:MLSUpsample expects radius,upsampleRadius,stepSize";
        VPC::ops::MlsUpsampleParams p;
        p.radius         = toFloat(parts[0], 0.1f);
        p.upsampleRadius = toFloat(parts[1], 0.05f);
        p.stepSize       = toFloat(parts[2], 0.01f);
        const auto o = VPC::ops::mlsUpsample(*world_, activeId(), p);
        if (!o.ok) return "Error:" + o.message;
        activeId() = o.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ConvexHull2D") {
        const auto r = VPC::ops::convexHull2D(*world_, activeId());
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["hullArea"] = std::to_string(r.area);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "ConcaveHull2D") {
        const auto parts = splitComma(arg);
        VPC::ops::ConcaveHullParams p;
        p.k    = parts.empty()      ? 3 : toInt(parts[0], 3);
        p.maxK = parts.size() < 2   ? 0 : toInt(parts[1], 0);
        const auto r = VPC::ops::concaveHull2D(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["hullArea"] = std::to_string(r.area);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    // --- Phase A: ICP / global (FPFH+RANSAC) registration ---

    if (name == "ICPAlign" || name == "ICPAlignPointToPlane") {
        const bool p2p = (name == "ICPAlignPointToPlane");
        const auto parts = splitComma(arg);
        if (parts.size() < 2)
            return p2p ? "Error:ICPAlignPointToPlane expects targetId,maxIterations"
                       : "Error:ICPAlign expects targetId,maxIterations";
        VPC::ops::IcpParams p;
        p.targetSceneId = toInt(parts[0], -1);
        p.maxIterations = toInt(parts[1], 50);
        p.pointToPlane  = p2p;
        const auto r = VPC::ops::icpAlign(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["fitness"]    = std::to_string(r.fitness);
        lastMetrics_["iterations"] = std::to_string(r.iterations);
        lastMetrics_["converged"]  = r.converged ? "Yes" : "No";
        if (!p2p) lastMetrics_["scale"] = std::to_string(r.scale);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    if (name == "GlobalRegister") {
        const auto parts = splitComma(arg);
        if (parts.size() < 3) return "Error:GlobalRegister expects targetId,k,iterations";
        VPC::ops::GlobalRegisterParams p;
        p.targetSceneId = toInt(parts[0], -1);
        p.fpfhK         = toInt(parts[1], 20);
        p.iterations    = toInt(parts[2], 1000);
        const auto r = VPC::ops::globalRegister(*world_, activeId(), p);
        if (!r.outcome.ok) return "Error:" + r.outcome.message;
        lastMetrics_["inlierCount"] = std::to_string(r.inlierCount);
        lastMetrics_["inlierRmse"]  = std::to_string(r.inlierRmse);
        activeId() = r.outcome.primarySceneId;
        if (onWorldChanged_) onWorldChanged_(activeId());
        return "Id:" + std::to_string(activeId());
    }

    return "Error:unknown command '" + cmd + "'";
}
