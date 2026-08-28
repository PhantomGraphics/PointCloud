#include "GSPointPresenter.h"

#include <algorithm>
#include <cmath>

namespace VPC {
namespace {

static glm::vec3 rowFromQuat0(const float* q) {
    const float x = q[1], y = q[2], z = q[3], w = q[0];
    return {
        1.0f - 2.0f * (y * y + z * z),
        2.0f * (x * y - z * w),
        2.0f * (x * z + y * w)
    };
}

static glm::vec3 rowFromQuat1(const float* q) {
    const float x = q[1], y = q[2], z = q[3], w = q[0];
    return {
        2.0f * (x * y + z * w),
        1.0f - 2.0f * (x * x + z * z),
        2.0f * (y * z - x * w)
    };
}

static glm::vec3 rowFromQuat2(const float* q) {
    const float x = q[1], y = q[2], z = q[3], w = q[0];
    return {
        2.0f * (x * z - y * w),
        2.0f * (y * z + x * w),
        1.0f - 2.0f * (x * x + y * y)
    };
}

static float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

static float shDcToColor(float sh) {
    constexpr float C0 = 0.28209479177387814f;
    return std::clamp(sh * C0 + 0.5f, 0.0f, 1.0f);
}

} // namespace

std::vector<GSSplat> GSPointPresenter::build(const Phantom::PointCloud::GSPointCloud& cloud) {
    std::vector<GSSplat> out;
    out.reserve(cloud.points.size());

    for (const auto& p : cloud.points) {
        const float sx = std::exp(p.scale[0]);
        const float sy = std::exp(p.scale[1]);
        const float sz = std::exp(p.scale[2]);
        const float maxScale = std::max({ sx, sy, sz });

        const glm::vec3 r0 = rowFromQuat0(p.rot);
        const glm::vec3 r1 = rowFromQuat1(p.rot);
        const glm::vec3 r2 = rowFromQuat2(p.rot);

        // vMatrix = maxScale * S^{-1} * R^T stored as columns for GLSL mat3.
        // The largest axis (maxScale) has col-length 1, so it fills the point sprite.
        // Smaller axes are proportionally narrower, and rotation is preserved.
        const float msx = maxScale / sx;
        const float msy = maxScale / sy;
        const float msz = maxScale / sz;

        GSSplat s{};
        s.centerSize = { p.x, p.y, p.z, maxScale * 100.0f };
        s.covRow0 = { r0.x * msx, r0.y * msy, r0.z * msz, 0.0f };
        s.covRow1 = { r1.x * msx, r1.y * msy, r1.z * msz, 0.0f };
        s.covRow2 = { r2.x * msx, r2.y * msy, r2.z * msz, 0.0f };
        s.color = {
            shDcToColor(p.f_dc[0]),
            shDcToColor(p.f_dc[1]),
            shDcToColor(p.f_dc[2]),
            sigmoid(p.opacity)
        };

        out.push_back(s);
    }

    return out;
}

} // namespace VPC
