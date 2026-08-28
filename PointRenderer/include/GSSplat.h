#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <cstdint>

namespace VKR {

struct GSSplat {
    glm::vec4 centerSize;
    glm::vec4 covRow0;
    glm::vec4 covRow1;
    glm::vec4 covRow2;
    glm::vec4 color;
};

static_assert(sizeof(GSSplat) == 80, "GSSplat layout must remain binary-compatible");

struct GSSortPushConstants {
    uint32_t count = 0;
    uint32_t phase = 0;
    uint32_t _pad0 = 0;
    uint32_t _pad1 = 0;
    glm::vec4 eye = glm::vec4(0.f);
};

} // namespace VKR
