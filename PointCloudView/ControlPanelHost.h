#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#include "ControlPage.h"
#include "IEmbeddedPanel.h"

#include <array>
#include <functional>
#include <string>

namespace VPC {

// The single shared "Control" window
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 3/4).
//
// Holds one active ControlPage, a page<->panel table, an optional common status
// area drawn above every page, and renders the selected page's embedded panel.
// It keeps only non-owning pointers -- the panels are owned by PointCloudApp and
// outlive the host.
class ControlPanelHost : public ::VKG::IVkUIPanel {
public:
    void registerPage(ControlPage page, IEmbeddedPanel* panel);
    void setPage(ControlPage page);
    ControlPage getPage() const { return activePage_; }
    bool isPageRegistered(ControlPage page) const;

    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    // Drawn at the top of the window, before the active page, on every frame.
    void setStatusDrawer(std::function<void()> fn) { statusDrawer_ = std::move(fn); }

    // Lets the layout file also carry the Processing page's selected process id
    // (section 5). getter returns the current id (int cast of ProcessId),
    // setter applies a restored one. Both may be empty.
    void setProcessAccessors(std::function<int()> getter,
                             std::function<void(int)> setter)
    {
        processGetter_ = std::move(getter);
        processSetter_ = std::move(setter);
    }

    // Persists the last active page + window-visible flag to a small ini file
    // (Phase 3). Position/size and section open/closed state are left to
    // Dear ImGui's own imgui.ini. Pass {} to disable persistence (validation
    // captures).
    void setLayoutFile(std::string path) { layoutPath_ = std::move(path); }

    // Restores page + visibility to first-run defaults, without touching
    // imgui.ini (Phase 3 "Reset Layout").
    void resetLayout();

    // Pins the Control window to a fixed position/size every frame (instead of
    // ImGuiCond_Once) so validation captures are reproducible regardless of any
    // imgui.ini on disk.
    void setFixedLayout(bool v) { fixedLayout_ = v; }

    void onImGui() override;

private:
    void loadLayout();
    void saveLayoutIfChanged();

    std::array<IEmbeddedPanel*, kControlPageCount> panels_{};
    ControlPage activePage_ = ControlPage::Scenes;
    bool visible_ = true;
    bool fixedLayout_ = false;
    std::function<void()> statusDrawer_;
    std::function<int()>  processGetter_;
    std::function<void(int)> processSetter_;

    std::string layoutPath_;
    bool        layoutLoaded_ = false;
    ControlPage lastSavedPage_ = ControlPage::Scenes;
    bool        lastSavedVisible_ = true;
    int         lastSavedProcess_ = -1;
};

} // namespace VPC
