#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class DBSCANClusteringView : public IProcessView {
public:
    const char* getName() const override { return "DBSCAN Clustering"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    float eps_    = 0.02f;
    int   minPts_ = 10;
    bool                hasResult_ = false;
    ops::ClusterResult  result_;
};

} // namespace VPC
