#include "DownSamplerView.h"

#include "DownSampler.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void DownSamplerView::onImGui(World& world, int activeSceneId,
                               const std::function<void()>& onRebuild)
{
    runButton_.setFunction([&world, activeSceneId, &onRebuild, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr || cellSize_ <= 0.0f) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::DownSampler downSampler;
        for (const auto& p : positions) {
            downSampler.add(p);
        }
        downSampler.execute(cellSize_);

        const auto sampled = downSampler.getDownSampled();
        if (sampled.empty()) return;

        const float ratio = static_cast<float>(sampled.size()) /
                            static_cast<float>(positions.size());

        auto* result = world.addScene("DownSampleResult");
        for (const auto& p : sampled) {
            result->add(p, glm::vec3(ratio, ratio, ratio));
        }

        scene->setVisible(false);
        onRebuild();
    });

    ImGui::SliderFloat("Cell Size", &cellSize_, 0.001f, 1.0f, "%.4f");
    runButton_.show();
}

} // namespace VPC
