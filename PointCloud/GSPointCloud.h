#pragma once
#include "GSPoint.h"

#include <cstdint>

namespace Phantom
{
    namespace PointCloud
    {
        /// @brief Manages a Gaussian Splatting point cloud.
        struct GSPointCloud
        {
        public:
            std::vector<GSPoint> points;  ///< The list of Gaussian Splatting points.

            /// @brief Monotonically increasing data-generation id, assigned by readFromFile()
            /// on every successful load (drawn from a process-global counter, so two consecutive
            /// loads always differ even when they contain the same number of splats). GPU consumers
            /// (e.g. GSComputePBVR) key their input-buffer cache on this instead of points.size(),
            /// so reloading a different file with an identical splat count still refreshes the SSBO.
            /// 0 means "never loaded".
            std::uint64_t generation = 0;

            /// @brief Loads the point cloud from a PLY (standard or SuperSplat-compressed) or .splat file.
            /// Format is selected by the file extension (".splat" vs. everything else treated as PLY).
            /// @param filename Path to the file to read.
            /// @return true on success, false on failure.
            bool readFromFile(const std::string& filename);
        };

    }
}
