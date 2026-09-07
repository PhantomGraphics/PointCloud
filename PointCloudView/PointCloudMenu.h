#pragma once

#include "ControlPage.h"
#include "ProcessRegistry.h"

namespace VPC {

class ControlPanelHost;
class ProcessPanel;

// Draws the "PointCloud" main-menu category. It only selects a ControlPage (and,
// for algorithm entries, a ProcessId on the Processing page) -- it never touches
// the World or creates a panel
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 4). Call
// onImGuiMenuBar() inside an ImGui::BeginMainMenuBar() scope.
class PointCloudMenu {
public:
    void init(ControlPanelHost* host, ProcessPanel* processPanel);

    void onImGuiMenuBar();

private:
    void pageItem(ControlPage page);
    void processCategory(ProcessCategory category);

    ControlPanelHost* host_         = nullptr;
    ProcessPanel*     processPanel_ = nullptr;
};

} // namespace VPC
