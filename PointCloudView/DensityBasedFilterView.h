#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class DensityBasedFilterView : public IProcessView {
public:
    const char* getName() const override { return "Density Filter"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_ = 0.05f;
    ops::ProcessOutcome lastOutcome_;
};

} // namespace VPC
