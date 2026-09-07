#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class DownSamplerView : public IProcessView {
public:
    const char* getName() const override { return "Down Sampler"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float               cellSize_ = ops::DownSampleParams{}.cellSize;
    ops::ProcessOutcome lastOutcome_;
};

} // namespace VPC
