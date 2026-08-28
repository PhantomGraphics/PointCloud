#include "../include/VkPointScene.h"

#include <algorithm>

namespace VKR {

int VkPointScene::add(const std::string& name) {
    SceneData data;
    data.id = nextId_++;
    data.name = name;
    scenes_.push_back(std::move(data));
    return scenes_.back().id;
}

int VkPointScene::add(const std::string& name, int id) {
    SceneData data;
    data.id = id;
    data.name = name;
    scenes_.push_back(std::move(data));
    nextId_ = std::max(nextId_, id + 1);
    return id;
}

void VkPointScene::remove(int id) {
    scenes_.erase(std::remove_if(scenes_.begin(), scenes_.end(),
        [id](const SceneData& s) { return s.id == id; }), scenes_.end());
}

void VkPointScene::clear() {
    scenes_.clear();
}

void VkPointScene::addPoints(int id,
                             const std::vector<glm::vec3>& positions,
                             const std::vector<glm::vec3>& colors) {
    auto* s = find(id);
    if (!s) return;

    const size_t n = std::min(positions.size(), colors.size());
    s->positions.insert(s->positions.end(), positions.begin(), positions.begin() + n);
    s->colors.insert(s->colors.end(), colors.begin(), colors.begin() + n);
}

void VkPointScene::setNormals(int id, const std::vector<glm::vec3>& normals) {
    auto* s = find(id);
    if (!s) return;
    s->normals = normals;
}

void VkPointScene::setGSSplats(int id, const std::vector<GSSplat>& splats) {
    auto* s = find(id);
    if (!s) return;
    s->gsSplats = splats;
}

void VkPointScene::setVisible(int id, bool visible) {
    auto* s = find(id);
    if (!s) return;
    s->visible = visible;
}

const VkPointScene::SceneData* VkPointScene::find(int id) const {
    for (const auto& s : scenes_) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

VkPointScene::SceneData* VkPointScene::find(int id) {
    for (auto& s : scenes_) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

std::vector<int> VkPointScene::allIds() const {
    std::vector<int> ids;
    ids.reserve(scenes_.size());
    for (const auto& s : scenes_) ids.push_back(s.id);
    return ids;
}

} // namespace VKR
