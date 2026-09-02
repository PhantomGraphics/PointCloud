#pragma once
#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Box3d.h"
#include <string>
#include <vector>

namespace Phantom { namespace PC {

// SoA (Structure of Arrays) point cloud: per-point attributes are held as parallel arrays
// rather than an array of per-point structs/heap objects. This is the sole point-cloud
// container for Phantom::PC going forward - it replaced the old AoS `PointCloud<T>` template
// and `PointXYZ`/`PointXYZf`/`PointXYZRGBf`/`PointXYZRGBNormal` point types, which offered no
// benefit over parallel arrays since none of the core algorithms consumed them directly.
//
// `colors`/`normals`/`scalars` are optional parallel arrays (same size as `positions`, or
// empty when unused). `normals` is populated by estimators such as NormalEstimator;
// loadPointCloud()/savePointCloud() leave it empty since none of the PCD/PLY/TXT readers or
// writers persist normals yet. `scalars` holds a single generic per-point value (e.g. the old PointXYZf::value).
struct PointCloudColoredData {
    std::vector<Math::Vector3df> positions;
    std::vector<Math::Vector3df> colors;
    std::vector<Math::Vector3df> normals;
    std::vector<float> scalars;
    bool   empty() const { return positions.empty(); }
    size_t size()  const { return positions.size(); }
    bool   hasColors()  const { return colors.size()  == positions.size() && !colors.empty(); }
    bool   hasNormals() const { return normals.size() == positions.size() && !normals.empty(); }
    bool   hasScalars() const { return scalars.size() == positions.size() && !scalars.empty(); }

    Math::Box3df getBoundingBox() const {
        auto bb = Math::Box3d<float>::createDegeneratedBox();
        for (const auto& p : positions) bb.add(p);
        return bb;
    }
};

// Load from file; dispatches by extension (.pcd, .ply, .txt).
// Returns false and sets errorMessage on failure.
bool loadPointCloud(const std::string& path, PointCloudColoredData& out, std::string& errorMessage);

// Save to file; dispatches by extension (.pcd, .ply, .txt).
// Returns false and sets errorMessage on failure.
bool savePointCloud(const std::string& path, const PointCloudColoredData& data, std::string& errorMessage);

}} // namespace Phantom::PC
