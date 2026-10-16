#pragma once

#include "GSSplat.h"

#include "../Core/GSPointCloud.h"

#include <vector>

namespace Phantom::VPC {

class GSPointPresenter {
public:
    static std::vector<GSSplat> build(const Phantom::PointCloud::GSPointCloud& cloud);
};

} // namespace VPC
