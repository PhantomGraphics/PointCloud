#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <string>
#include <vector>

#include "GSSplat.h"

namespace VPC {

/// @brief Single-precision point-cloud scene with position/color data.
class PointCloudfScene {
public:
    PointCloudfScene(int id, const std::string& name)
        : id_(id), name_(name) {}

    int         getId()   const { return id_; }
    std::string getName() const { return name_; }
    void        setName(const std::string& n) { name_ = n; }

    bool isVisible() const  { return visible_; }
    void setVisible(bool v) { visible_ = v; }

    void add(const glm::vec3& pos, const glm::vec3& color) {
        positions_.push_back(pos);
        colors_.push_back(color);
    }

    void clear() { positions_.clear(); colors_.clear(); normals_.clear(); }

    size_t getSize() const { return positions_.size(); }

    const std::vector<glm::vec3>& getPositions() const { return positions_; }
    const std::vector<glm::vec3>& getColors()    const { return colors_; }

    void setNormals(std::vector<glm::vec3> normals) { normals_ = std::move(normals); }
    const std::vector<glm::vec3>& getNormals() const { return normals_; }
    bool hasNormals() const { return !normals_.empty(); }

    void setGSSplats(std::vector<GSSplat> splats) { gsSplats_ = std::move(splats); }
    const std::vector<GSSplat>& getGSSplats() const { return gsSplats_; }
    bool hasGSSplats() const { return !gsSplats_.empty(); }

private:
    int         id_;
    std::string name_;
    bool        visible_ = true;
    std::vector<glm::vec3> positions_;
    std::vector<glm::vec3> colors_;
    std::vector<glm::vec3> normals_;
    std::vector<GSSplat> gsSplats_;
};

} // namespace VPC
