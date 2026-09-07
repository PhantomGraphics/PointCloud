#include "ImportExportPanel.h"

#include "PointCloudRenderer.h"
#include "FileOpenDialog.h"
#include "FileSaveDialog.h"
#include "imgui.h"

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

void ImportExportPanel::init(PointCloudRenderer* renderer, IoFn onLoad, IoFn onSave)
{
    renderer_ = renderer;
    onLoad_   = std::move(onLoad);
    onSave_   = std::move(onSave);
}

void ImportExportPanel::drawContents()
{
    ImGui::TextUnformatted("Import");
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
    ImGui::Separator();
    ImGui::TextUnformatted("Export");

    const bool hasPoints = renderer_ && renderer_->getPointCount() > 0;
    if (!hasPoints) {
        ImGui::TextDisabled("Nothing to export (the visible point count is 0).");
        ImGui::BeginDisabled();
    }
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
