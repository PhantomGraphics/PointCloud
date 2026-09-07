#pragma once

#include "IProcessView.h"
#include "PointCloudOps.h"

namespace VPC {

class GroundExtractorView : public IProcessView {
public:
    const char* getName() const override { return "Ground Extractor"; }
    void onImGui(World& world, int activeSceneId,
                 const std::function<void(int)>& onResult) override;

private:
    ops::GroundParams params_;
    bool              hasResult_ = false;
    ops::GroundResult result_;
};

} // namespace VPC
