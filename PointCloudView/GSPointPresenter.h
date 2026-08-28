#pragma once

#include "GSSplat.h"

#include "../PointCloud/GSPointCloud.h"

#include <vector>

namespace VPC {

class GSPointPresenter {
public:
    static std::vector<GSSplat> build(const Phantom::PointCloud::GSPointCloud& cloud);
};

} // namespace VPC
