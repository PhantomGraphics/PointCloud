#pragma once

#include <cstddef>

namespace VPC {

// The operation page currently shown in the single shared "Control" window.
//
// Exactly one page is active at a time. The PointCloud main menu selects it,
// ControlPanelHost renders it. This mirrors PhysicsView's ControlPage, but is
// kept PointCloudView-local on purpose (see
// docs/todo/PLAN_pointcloudview_gui_restructuring.md section 4).
enum class ControlPage {
    Scenes = 0,
    Rendering,
    Processing,
    ImportExport,
    ScenarioBrowser,
    Count,
};

inline constexpr std::size_t kControlPageCount =
    static_cast<std::size_t>(ControlPage::Count);

inline const char* toString(ControlPage page)
{
    switch (page) {
    case ControlPage::Scenes:          return "Scenes";
    case ControlPage::Rendering:       return "Rendering";
    case ControlPage::Processing:      return "Processing";
    case ControlPage::ImportExport:    return "Import / Export";
    case ControlPage::ScenarioBrowser: return "Scenario Browser";
    default:                           return "?";
    }
}

} // namespace VPC
