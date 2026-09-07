#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class DistanceBasedClusteringView : public IProcessView {
public:
    const char* getName() const override { return "Region Growing"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float searchRadius_ = 0.02f;
    bool               hasResult_ = false;
    ops::ClusterResult result_;
};

} // namespace VPC
