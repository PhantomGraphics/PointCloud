#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace Phantom::VPC {

class ConcaveHull2dView : public IProcessView {
public:
    const char* getName() const override { return "Concave Hull 2D"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    int k_    = 3;
    int maxK_ = 0; // 0 = "as many as needed"

    bool            hasResult_ = false;
    ops::HullResult result_;
};

} // namespace VPC
