#include "Menu.h"
#include "SceneListPanel.h"
#include "FileOpenDialog.h"
#include "FileSaveDialog.h"
#include "imgui.h"

#include "DensityEstimatorView.h"
#include "NormalEstimatorView.h"
#include "CurvatureEstimatorView.h"
#include "DownSamplerView.h"
#include "RansacPlaneDetectorView.h"
#include "RansacCylinderDetectorView.h"
#include "RansacSphereDetectorView.h"
#include "RansacConeDetectorView.h"
#include "DBSCANClusteringView.h"
#include "DistanceBasedClusteringView.h"
#include "RegionGrowingView.h"
#include "DensityBasedFilterView.h"
#include "CurvatureBasedFilterView.h"
#include "GroundExtractorView.h"
#include "FPFHEstimatorView.h"
#include "BoundaryDetectorView.h"
#include "ICPRegistrationView.h"
#include "GlobalRegistrationView.h"
#include "MLSSurfaceView.h"
#include "ConvexHull2DView.h"
#include "ConcaveHull2DView.h"
#include "PCSphereView.h"
#include "PCCylinderView.h"
#include "PCRectView.h"
#include "GreedyProjectionMeshGeneratorView.h"
#include "PoissonSurfaceView.h"

#include <cstdio>

namespace VPC {

namespace {

bool chooseImportPath(std::array<char, 512>& path) {
    Phantom::UI::FileOpenDialog dlg("Import Point Cloud");
    dlg.addFilter("*.pcd"); dlg.addFilter("*.ply"); dlg.addFilter("*.txt");
    dlg.show();
    const auto sel = dlg.getFilePath();
    if (sel.empty()) return false;
    std::snprintf(path.data(), path.size(), "%s", sel.string().c_str());
    return true;
}

bool chooseExportPath(std::array<char, 512>& path) {
    Phantom::UI::FileSaveDialog dlg("Export Point Cloud");
    dlg.addFilter("*.pcd"); dlg.addFilter("*.ply"); dlg.addFilter("*.txt");
    dlg.show();
    const auto sel = dlg.getFilePath();
    if (sel.empty()) return false;
    std::snprintf(path.data(), path.size(), "%s", sel.string().c_str());
    return true;
}

} // namespace

void Menu::init(
    World* world, int* pActiveSceneId,
    std::function<void()> onWorldChanged,
    PointCloudRenderer::RenderMode* pRenderMode,
    SceneListPanel* scenePanel,
    PointCloudRenderer* renderer,
    std::function<bool(const std::string&, std::string&)> onLoad,
    std::function<bool(const std::string&, std::string&)> onSave)
{
    world_          = world;
    pId_            = pActiveSceneId;
    pRenderMode_    = pRenderMode;
    onWorldChanged_ = std::move(onWorldChanged);
    scenePanel_     = scenePanel;
    renderer_       = renderer;
    onLoad_         = std::move(onLoad);
    onSave_         = std::move(onSave);
}

// ============================================================
//  IVkUIPanel
// ============================================================

void Menu::onImGui() {
    ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(430.f, 620.f), ImGuiCond_Once);
    if (!ImGui::Begin("Control")) { ImGui::End(); return; }

    if (renderer_) {
        ImGui::Text("Total points: %u", renderer_->getPointCount());
        ImGui::Spacing();
    }

    if (scenePanel_) scenePanel_->onImGui();

    ImGui::Spacing();
    ImGui::Separator();

    if (renderer_) renderer_->drawImGuiControls();

    drawProcessView();
    drawImportExport();

    ImGui::End();
}

// ============================================================
//  Menu bar
// ============================================================

void Menu::onImGuiMenuBar() {
    if (!ImGui::BeginMenu("PointCloud")) return;

    auto setView = [this](IProcessView* v) { activeProcessView_.reset(v); };

    if (pRenderMode_) {
        const bool point = (*pRenderMode_ == PointCloudRenderer::RenderMode::Point);
        if (ImGui::MenuItem("RenderPoint", nullptr, point))
            *pRenderMode_ = PointCloudRenderer::RenderMode::Point;

        const bool gauss = (*pRenderMode_ == PointCloudRenderer::RenderMode::GaussianSplatting);
        if (ImGui::MenuItem("RenderGaussian", nullptr, gauss))
            *pRenderMode_ = PointCloudRenderer::RenderMode::GaussianSplatting;

        ImGui::Separator();
    }

    if (ImGui::MenuItem("Density Estimator"))       setView(new DensityEstimatorView());
    if (ImGui::MenuItem("Normal Estimator"))         setView(new NormalEstimatorView());
    if (ImGui::MenuItem("Curvature Estimator"))      setView(new CurvatureEstimatorView());
    if (ImGui::MenuItem("Down Sampler"))             setView(new DownSamplerView());
    if (ImGui::MenuItem("FPFH Estimator"))           setView(new FPFHEstimatorView());
    if (ImGui::MenuItem("Boundary Detector"))        setView(new BoundaryDetectorView());
    ImGui::Separator();
    if (ImGui::MenuItem("RANSAC Plane Detector"))    setView(new RansacPlaneDetectorView());
    if (ImGui::MenuItem("RANSAC Cylinder Detector")) setView(new RansacCylinderDetectorView());
    if (ImGui::MenuItem("RANSAC Sphere Detector"))   setView(new RansacSphereDetectorView());
    if (ImGui::MenuItem("RANSAC Cone Detector"))     setView(new RansacConeDetectorView());
    ImGui::Separator();
    if (ImGui::MenuItem("DBSCAN Clustering"))              setView(new DBSCANClusteringView());
    if (ImGui::MenuItem("Region Growing (Distance)"))      setView(new DistanceBasedClusteringView());
    if (ImGui::MenuItem("Region Growing (Normal/Curvature)")) setView(new RegionGrowingView());
    if (ImGui::MenuItem("Ground Extractor"))               setView(new GroundExtractorView());
    ImGui::Separator();
    if (ImGui::MenuItem("Density Filter"))           setView(new DensityBasedFilterView());
    if (ImGui::MenuItem("Curvature Filter"))         setView(new CurvatureBasedFilterView());
    ImGui::Separator();
    if (ImGui::MenuItem("ICP Registration"))             setView(new ICPRegistrationView());
    if (ImGui::MenuItem("Global Registration (FPFH+RANSAC)")) setView(new GlobalRegistrationView());
    ImGui::Separator();
    if (ImGui::MenuItem("Generate Sphere"))          setView(new PCSphereView());
    if (ImGui::MenuItem("Generate Cylinder"))        setView(new PCCylinderView());
    if (ImGui::MenuItem("Generate Rect"))            setView(new PCRectView());
    ImGui::Separator();
    if (ImGui::MenuItem("Greedy Projection Mesh"))   setView(new GreedyProjectionMeshGeneratorView());
    if (ImGui::MenuItem("Poisson Surface"))          setView(new PoissonSurfaceView());
    if (ImGui::MenuItem("MLS Surface"))              setView(new MLSSurfaceView());
    if (ImGui::MenuItem("Convex Hull 2D"))           setView(new ConvexHull2DView());
    if (ImGui::MenuItem("Concave Hull 2D"))          setView(new ConcaveHull2DView());

    ImGui::EndMenu();
}

// ============================================================
//  Private helpers
// ============================================================

void Menu::drawProcessView() {
    if (!activeProcessView_) return;

    ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f),
                       "[%s]", activeProcessView_->getName());
    ImGui::SameLine();
    if (ImGui::SmallButton("Close")) {
        activeProcessView_.reset();
    } else {
        ImGui::Spacing();
        if (world_ && pId_) {
            activeProcessView_->onImGui(*world_, *pId_,
                [this]() { if (onWorldChanged_) onWorldChanged_(); });
        }
    }
    ImGui::Spacing();
    ImGui::Separator();
}

void Menu::drawImportExport() {
    if (!ImGui::CollapsingHeader("Import / Export")) return;

    if (ImGui::Button("Browse Import...")) chooseImportPath(importPath_);
    ImGui::InputText("Import path", importPath_.data(), importPath_.size());
    if (ImGui::Button("Import (.pcd/.ply/.txt)") && onLoad_) {
        std::string err;
        if (onLoad_(importPath_.data(), err))
            statusMessage_ = "Imported: " + std::string(importPath_.data());
        else
            statusMessage_ = "Import failed: " + err;
    }

    ImGui::Spacing();

    const bool hasPoints = renderer_ && renderer_->getPointCount() > 0;
    if (!hasPoints) ImGui::BeginDisabled();
    if (ImGui::Button("Browse Export...")) chooseExportPath(exportPath_);
    ImGui::InputText("Export path", exportPath_.data(), exportPath_.size());
    if (ImGui::Button("Export (.pcd/.ply/.txt)") && onSave_) {
        std::string err;
        if (onSave_(exportPath_.data(), err))
            statusMessage_ = "Exported: " + std::string(exportPath_.data());
        else
            statusMessage_ = "Export failed: " + err;
    }
    if (!hasPoints) ImGui::EndDisabled();

    if (!statusMessage_.empty()) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextWrapped("%s", statusMessage_.c_str());
    }
}

} // namespace VPC
