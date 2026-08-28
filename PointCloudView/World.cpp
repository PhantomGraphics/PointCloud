#include "World.h"
#include <algorithm>

namespace VPC {

PointCloudfScene* World::addScene(const std::string& name) {
    auto scene = std::make_unique<PointCloudfScene>(nextId_++, name);
    auto* ptr  = scene.get();
    scenes_.push_back(std::move(scene));
    return ptr;
}

void World::removeScene(int id) {
    scenes_.erase(
        std::remove_if(scenes_.begin(), scenes_.end(),
            [id](const std::unique_ptr<PointCloudfScene>& s) {
                return s->getId() == id;
            }),
        scenes_.end());
}

PointCloudfScene* World::findById(int id) {
    for (auto& s : scenes_) {
        if (s->getId() == id) return s.get();
    }
    return nullptr;
}

void World::clear() {
    scenes_.clear();
}

void World::clearPolygons() {
    polygons_.clear();
}

void World::addPolygon(PolygonMesh mesh) {
    polygons_.push_back(std::move(mesh));
}

} // namespace VPC
