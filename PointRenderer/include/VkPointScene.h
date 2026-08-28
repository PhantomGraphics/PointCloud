#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "GSSplat.h"

#include <string>
#include <vector>

namespace VKR {

class VkPointScene {
public:
    struct SceneData {
        int id = -1;
        std::string name;
        bool visible = true;
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> colors;
        std::vector<glm::vec3> normals;
        std::vector<GSSplat> gsSplats;
    };

    int add(const std::string& name);
    int add(const std::string& name, int id);
    void remove(int id);
    void clear();

    void addPoints(int id,
                   const std::vector<glm::vec3>& positions,
                   const std::vector<glm::vec3>& colors);
    void setNormals(int id, const std::vector<glm::vec3>& normals);
    void setGSSplats(int id, const std::vector<GSSplat>& splats);
    void setVisible(int id, bool visible);

    const SceneData* find(int id) const;
    SceneData*       find(int id);
    std::vector<int> allIds() const;

private:
    int nextId_ = 0;
    std::vector<SceneData> scenes_;
};

} // namespace VKR
