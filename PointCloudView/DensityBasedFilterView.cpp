#include "DensityBasedFilterView.h"

#include "DensityBasedFilter.h"
#include "imgui.h"

#include <glm/glm.hpp>

namespace VPC {

void DensityBasedFilterView::onImGui(World& world, int activeSceneId,
                                      const std::function<void(int)>& onResult)
{
    runButton_.setFunction([&world, activeSceneId, &onResult, this]() {
        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) return;

        const auto& positions = scene->getPositions();

        Phantom::PC::DensityBasedFilter filter;
        for (const auto& p : positions) {
            filter.add(p);
        }
        filter.execute(searchRadius_);

        const auto inlierIndices = filter.getInlierIndices();

        auto* result = world.addScene("DensityFilterResult");
        for (const auto idx : inlierIndices) {
            if (idx >= 0 && static_cast<size_t>(idx) < positions.size()) {
                result->add(positions[idx], glm::vec3(0.6f, 0.85f, 1.0f));
            }
        }

        scene->setVisible(false);
        onResult(-1);
    });

    ImGui::SliderFloat("Search Radius", &searchRadius_, 0.001f, 1.0f, "%.4f");
    runButton_.show();
}

} // namespace VPC
