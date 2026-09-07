#include "PointCloudOps.h"

#include "World.h"
#include "PointCloudfScene.h"

#include "DownSampler.h"

#include <glm/glm.hpp>

namespace VPC {
namespace ops {

ProcessOutcome downSample(World& world, int activeSceneId, const DownSampleParams& p)
{
    ProcessOutcome out;

    auto* scene = world.findById(activeSceneId);
    if (!scene) {
        out.message = "no active scene";
        return out;
    }
    if (p.cellSize <= 0.f) {
        out.message = "invalid cell size";
        return out;
    }

    const auto& positions = scene->getPositions();

    Phantom::PC::DownSampler downSampler;
    for (const auto& q : positions) downSampler.add(q);
    downSampler.execute(p.cellSize);

    const auto sampled = downSampler.getDownSampled();
    if (sampled.empty()) {
        out.message = "downsample produced empty result";
        return out;
    }

    const float ratio = static_cast<float>(sampled.size()) /
                        static_cast<float>(positions.size());

    auto* result = world.addScene("DownSampleResult");
    for (const auto& q : sampled)
        result->add(q, glm::vec3(ratio, ratio, ratio));

    scene->setVisible(false);

    out.ok            = true;
    out.resultSceneId = result->getId();
    out.message       = "Downsampled " + std::to_string(positions.size()) +
                        " -> " + std::to_string(sampled.size()) + " points";
    return out;
}

} // namespace ops
} // namespace VPC
