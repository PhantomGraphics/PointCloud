#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <cstdint>

namespace VPC {

// One element of Gaussian Splatting on the Vulkan side.
// Laid out in vec4 units for compatibility with both std430 and vertex attributes.
struct GSSplat {
    glm::vec4 centerSize; // xyz: center, w: point size
    glm::vec4 covRow0;    // covariance row 0 (scaled rotation row)
    glm::vec4 covRow1;    // covariance row 1
    glm::vec4 covRow2;    // covariance row 2
    glm::vec4 color;      // rgba
};

struct GSSortPushConstants {
    uint32_t count = 0;
    uint32_t phase = 0;
    uint32_t _pad0 = 0;
    uint32_t _pad1 = 0;
    glm::vec4 eye = glm::vec4(0.f);
};

} // namespace VPC
