#pragma once
#include "GSPoint.h"

namespace Phantom
{
    namespace PointCloud
    {
        /// @brief Manages a Gaussian Splatting point cloud.
        struct GSPointCloud
        {
        public:
            std::vector<GSPoint> points;  ///< The list of Gaussian Splatting points.

            /// @brief Loads the point cloud from a PLY (standard or SuperSplat-compressed) or .splat file.
            /// Format is selected by the file extension (".splat" vs. everything else treated as PLY).
            /// @param filename Path to the file to read.
            /// @return true on success, false on failure.
            bool readFromFile(const std::string& filename);
        };

    }
}
