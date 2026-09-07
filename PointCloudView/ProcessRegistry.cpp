#include "ProcessRegistry.h"

#include "PCSphereView.h"
#include "PCCylinderView.h"
#include "PCRectView.h"
#include "DensityEstimatorView.h"
#include "NormalEstimatorView.h"
#include "CurvatureEstimatorView.h"
#include "FPFHEstimatorView.h"
#include "BoundaryDetectorView.h"
#include "DownSamplerView.h"
#include "DensityBasedFilterView.h"
#include "CurvatureBasedFilterView.h"
#include "DBSCANClusteringView.h"
#include "DistanceBasedClusteringView.h"
#include "RegionGrowingView.h"
#include "GroundExtractorView.h"
#include "RansacPlaneDetectorView.h"
#include "RansacCylinderDetectorView.h"
#include "RansacSphereDetectorView.h"
#include "RansacConeDetectorView.h"
#include "ICPRegistrationView.h"
#include "GlobalRegistrationView.h"
#include "GreedyProjectionMeshGeneratorView.h"
#include "PoissonSurfaceView.h"
#include "MLSSurfaceView.h"
#include "ConvexHull2DView.h"
#include "ConcaveHull2DView.h"

namespace VPC {

const char* processName(ProcessId id)
{
    switch (id) {
    case ProcessId::GenerateSphere:       return "Generate Sphere";
    case ProcessId::GenerateCylinder:     return "Generate Cylinder";
    case ProcessId::GenerateRect:         return "Generate Rect";
    case ProcessId::DensityEstimator:     return "Density Estimator";
    case ProcessId::NormalEstimator:      return "Normal Estimator";
    case ProcessId::CurvatureEstimator:   return "Curvature Estimator";
    case ProcessId::FPFHEstimator:        return "FPFH Estimator";
    case ProcessId::BoundaryDetector:     return "Boundary Detector";
    case ProcessId::DownSampler:          return "Down Sampler";
    case ProcessId::DensityFilter:        return "Density Filter";
    case ProcessId::CurvatureFilter:      return "Curvature Filter";
    case ProcessId::DBSCANClustering:     return "DBSCAN Clustering";
    case ProcessId::DistanceClustering:   return "Region Growing (Distance)";
    case ProcessId::RegionGrowing:        return "Region Growing (Normal/Curvature)";
    case ProcessId::GroundExtractor:      return "Ground Extractor";
    case ProcessId::RansacPlane:          return "RANSAC Plane Detector";
    case ProcessId::RansacCylinder:       return "RANSAC Cylinder Detector";
    case ProcessId::RansacSphere:         return "RANSAC Sphere Detector";
    case ProcessId::RansacCone:           return "RANSAC Cone Detector";
    case ProcessId::ICPRegistration:      return "ICP Registration";
    case ProcessId::GlobalRegistration:   return "Global Registration (FPFH+RANSAC)";
    case ProcessId::GreedyProjectionMesh: return "Greedy Projection Mesh";
    case ProcessId::PoissonSurface:       return "Poisson Surface";
    case ProcessId::MLSSurface:           return "MLS Surface";
    case ProcessId::ConvexHull2D:         return "Convex Hull 2D";
    case ProcessId::ConcaveHull2D:        return "Concave Hull 2D";
    default:                              return "";
    }
}

bool processWorksOnEmptyWorld(ProcessId id)
{
    switch (id) {
    case ProcessId::GenerateSphere:
    case ProcessId::GenerateCylinder:
    case ProcessId::GenerateRect:
        return true;
    default:
        return false;
    }
}

bool processNeedsReferenceScene(ProcessId id)
{
    return id == ProcessId::ICPRegistration ||
           id == ProcessId::GlobalRegistration;
}

const char* categoryLabel(ProcessCategory category)
{
    switch (category) {
    case ProcessCategory::Generate:     return "Generate";
    case ProcessCategory::Features:     return "Features";
    case ProcessCategory::Filters:      return "Filters";
    case ProcessCategory::Segmentation: return "Segmentation";
    case ProcessCategory::Fitting:      return "Fitting";
    case ProcessCategory::Registration: return "Registration";
    case ProcessCategory::Surface:      return "Surface";
    default:                            return "?";
    }
}

const std::vector<ProcessMenuItem>& processesInCategory(ProcessCategory category)
{
    static const std::vector<ProcessMenuItem> kGenerate = {
        { ProcessId::GenerateSphere,   "Sphere" },
        { ProcessId::GenerateCylinder, "Cylinder" },
        { ProcessId::GenerateRect,     "Rect" },
    };
    static const std::vector<ProcessMenuItem> kFeatures = {
        { ProcessId::DensityEstimator,   "Density" },
        { ProcessId::NormalEstimator,    "Normal" },
        { ProcessId::CurvatureEstimator, "Curvature" },
        { ProcessId::FPFHEstimator,      "FPFH" },
        { ProcessId::BoundaryDetector,   "Boundary" },
    };
    static const std::vector<ProcessMenuItem> kFilters = {
        { ProcessId::DownSampler,     "Down Sample" },
        { ProcessId::DensityFilter,   "Density" },
        { ProcessId::CurvatureFilter, "Curvature" },
    };
    static const std::vector<ProcessMenuItem> kSegmentation = {
        { ProcessId::DBSCANClustering,   "DBSCAN" },
        { ProcessId::DistanceClustering, "Distance" },
        { ProcessId::RegionGrowing,      "Region Growing" },
        { ProcessId::GroundExtractor,    "Ground" },
    };
    static const std::vector<ProcessMenuItem> kFitting = {
        { ProcessId::RansacPlane,    "RANSAC Plane" },
        { ProcessId::RansacCylinder, "RANSAC Cylinder" },
        { ProcessId::RansacSphere,   "RANSAC Sphere" },
        { ProcessId::RansacCone,     "RANSAC Cone" },
    };
    static const std::vector<ProcessMenuItem> kRegistration = {
        { ProcessId::ICPRegistration,    "ICP" },
        { ProcessId::GlobalRegistration, "Global Registration" },
    };
    static const std::vector<ProcessMenuItem> kSurface = {
        { ProcessId::GreedyProjectionMesh, "Greedy" },
        { ProcessId::PoissonSurface,       "Poisson" },
        { ProcessId::MLSSurface,           "MLS" },
        { ProcessId::ConvexHull2D,         "Convex Hull" },
        { ProcessId::ConcaveHull2D,        "Concave Hull" },
    };
    static const std::vector<ProcessMenuItem> kEmpty;

    switch (category) {
    case ProcessCategory::Generate:     return kGenerate;
    case ProcessCategory::Features:     return kFeatures;
    case ProcessCategory::Filters:      return kFilters;
    case ProcessCategory::Segmentation: return kSegmentation;
    case ProcessCategory::Fitting:      return kFitting;
    case ProcessCategory::Registration: return kRegistration;
    case ProcessCategory::Surface:      return kSurface;
    default:                            return kEmpty;
    }
}

std::unique_ptr<IProcessView> makeProcessView(ProcessId id)
{
    switch (id) {
    case ProcessId::GenerateSphere:       return std::make_unique<PCSphereView>();
    case ProcessId::GenerateCylinder:     return std::make_unique<PCCylinderView>();
    case ProcessId::GenerateRect:         return std::make_unique<PCRectView>();
    case ProcessId::DensityEstimator:     return std::make_unique<DensityEstimatorView>();
    case ProcessId::NormalEstimator:      return std::make_unique<NormalEstimatorView>();
    case ProcessId::CurvatureEstimator:   return std::make_unique<CurvatureEstimatorView>();
    case ProcessId::FPFHEstimator:        return std::make_unique<FPFHEstimatorView>();
    case ProcessId::BoundaryDetector:     return std::make_unique<BoundaryDetectorView>();
    case ProcessId::DownSampler:          return std::make_unique<DownSamplerView>();
    case ProcessId::DensityFilter:        return std::make_unique<DensityBasedFilterView>();
    case ProcessId::CurvatureFilter:      return std::make_unique<CurvatureBasedFilterView>();
    case ProcessId::DBSCANClustering:     return std::make_unique<DBSCANClusteringView>();
    case ProcessId::DistanceClustering:   return std::make_unique<DistanceBasedClusteringView>();
    case ProcessId::RegionGrowing:        return std::make_unique<RegionGrowingView>();
    case ProcessId::GroundExtractor:      return std::make_unique<GroundExtractorView>();
    case ProcessId::RansacPlane:          return std::make_unique<RansacPlaneDetectorView>();
    case ProcessId::RansacCylinder:       return std::make_unique<RansacCylinderDetectorView>();
    case ProcessId::RansacSphere:         return std::make_unique<RansacSphereDetectorView>();
    case ProcessId::RansacCone:           return std::make_unique<RansacConeDetectorView>();
    case ProcessId::ICPRegistration:      return std::make_unique<ICPRegistrationView>();
    case ProcessId::GlobalRegistration:   return std::make_unique<GlobalRegistrationView>();
    case ProcessId::GreedyProjectionMesh: return std::make_unique<GreedyProjectionMeshGeneratorView>();
    case ProcessId::PoissonSurface:       return std::make_unique<PoissonSurfaceView>();
    case ProcessId::MLSSurface:           return std::make_unique<MLSSurfaceView>();
    case ProcessId::ConvexHull2D:         return std::make_unique<ConvexHull2DView>();
    case ProcessId::ConcaveHull2D:        return std::make_unique<ConcaveHull2DView>();
    default:                              return nullptr;
    }
}

} // namespace VPC
