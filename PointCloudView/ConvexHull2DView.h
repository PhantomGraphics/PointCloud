#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class ConvexHull2DView : public IProcessView {
public:
    const char* getName() const override { return "Convex Hull 2D"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    bool            hasResult_ = false;
    ops::HullResult result_;
};

} // namespace VPC
