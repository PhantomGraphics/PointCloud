#pragma once

#include "IProcessView.h"

#include <memory>
#include <vector>

namespace VPC {

// Every algorithm panel PointCloudView exposes, plus the menu category it is
// filed under. The PointCloud menu shows these as "category -> item" entries
// that only *select* a process (page = Processing); the algorithm itself runs
// from the Processing page's Run button, exactly as before.
enum class ProcessId {
    None = -1,

    GenerateSphere = 0,
    GenerateCylinder,
    GenerateRect,

    DensityEstimator,
    NormalEstimator,
    CurvatureEstimator,
    FPFHEstimator,
    BoundaryDetector,

    DownSampler,
    DensityFilter,
    CurvatureFilter,

    DBSCANClustering,
    DistanceClustering,
    RegionGrowing,
    GroundExtractor,

    RansacPlane,
    RansacCylinder,
    RansacSphere,
    RansacCone,

    ICPRegistration,
    GlobalRegistration,

    GreedyProjectionMesh,
    PoissonSurface,
    MLSSurface,
    ConvexHull2D,
    ConcaveHull2D,

    Count,
};

inline constexpr int kProcessCount = static_cast<int>(ProcessId::Count);

enum class ProcessCategory {
    Generate = 0,
    Features,
    Filters,
    Segmentation,
    Fitting,
    Registration,
    Surface,
    Count,
};

struct ProcessMenuItem {
    ProcessId   id;
    const char* label; // short label shown inside the submenu
};

// Display name of a process (matches IProcessView::getName()).
const char* processName(ProcessId id);

const char* categoryLabel(ProcessCategory category);

// Menu items filed under a category, in menu order.
const std::vector<ProcessMenuItem>& processesInCategory(ProcessCategory category);

// Lazily constructs the panel for a process. Returns nullptr for
// ProcessId::None / out-of-range.
std::unique_ptr<IProcessView> makeProcessView(ProcessId id);

} // namespace VPC
