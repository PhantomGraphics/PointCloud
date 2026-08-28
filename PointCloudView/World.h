#pragma once

#include "PointCloudfScene.h"

#include <memory>
#include <string>
#include <vector>

namespace VPC {

/// @brief Polygon mesh overlay (planes, surfaces, reconstruction results) — display-only struct.
struct PolygonMesh {
    std::string          name;
    std::vector<float>    positions; ///< Flat vertex positions (x,y,z), 3 floats per vertex.
    std::vector<float>    colors;    ///< Flat vertex colors (r,g,b,a), 4 floats per vertex.
    std::vector<uint32_t> indices;   ///< Triangle index list.
    bool visible = true;
};

/// @brief Container that manages multiple PointCloudfScene objects by ID.
///
/// Rewritten from WorldBase without OpenGL dependencies; Vulkan-only replacement.
class World {
public:
    /// @brief Add a new scene and return a pointer to it.
    PointCloudfScene* addScene(const std::string& name);

    /// @brief Remove the scene with the given ID.
    void removeScene(int id);

    /// @brief Find the scene with the given ID; returns nullptr if not found.
    PointCloudfScene* findById(int id);

    /// @brief Remove all scenes.
    void clear();

    bool isEmpty() const { return scenes_.empty(); }

    const std::vector<std::unique_ptr<PointCloudfScene>>& getScenes() const { return scenes_; }
    std::vector<std::unique_ptr<PointCloudfScene>>&       getScenes()       { return scenes_; }

    // ---- Polygon overlay management ----
    void clearPolygons();
    void addPolygon(PolygonMesh mesh);
    const std::vector<PolygonMesh>& getPolygons() const { return polygons_; }

private:
    int nextId_ = 0;
    std::vector<std::unique_ptr<PointCloudfScene>> scenes_;
    std::vector<PolygonMesh> polygons_;
};

} // namespace VPC
