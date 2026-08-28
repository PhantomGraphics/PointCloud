#pragma once

#include <vector>
#include <string>

namespace Phantom
{
    namespace PointCloud
    {
        /// @brief Represents a single Gaussian Splatting point.
        /// Holds position, normal, color (DC term), opacity, scale, and rotation.
        struct GSPoint
        {
            float x, y, z;       ///< 3D position of the point.
            float nx, ny, nz;    ///< Normal vector.

            float f_dc[3];       ///< DC term of RGB color (0th-order spherical harmonics coefficient).
            float opacity;       ///< Opacity value.

            float scale[3];      ///< Scale along each axis.
            float rot[4];        ///< Rotation quaternion (w, x, y, z).
        };

    }
}
