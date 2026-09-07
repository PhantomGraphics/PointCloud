#include "ControlPanelHost.h"

#include "imgui.h"

#include <fstream>
#include <string>

namespace VPC {

void ControlPanelHost::registerPage(ControlPage page, IEmbeddedPanel* panel)
{
    if (page == ControlPage::Count) return;
    panels_[static_cast<std::size_t>(page)] = panel;
}

void ControlPanelHost::setPage(ControlPage page)
{
    if (page == ControlPage::Count) return;
    activePage_ = page;
}

bool ControlPanelHost::isPageRegistered(ControlPage page) const
{
    if (page == ControlPage::Count) return false;
    return panels_[static_cast<std::size_t>(page)] != nullptr;
}

void ControlPanelHost::resetLayout()
{
    activePage_ = ControlPage::Scenes;
    visible_    = true;
    if (processSetter_) processSetter_(-1);  // ProcessId::None
}

void ControlPanelHost::loadLayout()
{
    layoutLoaded_ = true;
    if (layoutPath_.empty()) return;

    std::ifstream f(layoutPath_);
    if (!f) return;
    std::string key;
    int page = 0, vis = 1, proc = -1;
    bool gotPage = false, gotVis = false, gotProc = false;
    while (f >> key) {
        if (key == "page")         { f >> page; gotPage = true; }
        else if (key == "visible") { f >> vis;  gotVis = true; }
        else if (key == "process") { f >> proc; gotProc = true; }
    }
    if (gotPage && page >= 0 && page < static_cast<int>(kControlPageCount))
        activePage_ = static_cast<ControlPage>(page);
    if (gotVis)
        visible_ = (vis != 0);
    if (gotProc && processSetter_)
        processSetter_(proc);

    lastSavedPage_    = activePage_;
    lastSavedVisible_ = visible_;
    lastSavedProcess_ = processGetter_ ? processGetter_() : -1;
}

void ControlPanelHost::saveLayoutIfChanged()
{
    const int proc = processGetter_ ? processGetter_() : -1;
    if (activePage_ == lastSavedPage_ && visible_ == lastSavedVisible_ &&
        proc == lastSavedProcess_)
        return;
    lastSavedPage_    = activePage_;
    lastSavedVisible_ = visible_;
    lastSavedProcess_ = proc;
    if (layoutPath_.empty()) return;

    std::ofstream f(layoutPath_, std::ios::trunc);
    if (!f) return;
    f << "page " << static_cast<int>(activePage_) << '\n'
      << "visible " << (visible_ ? 1 : 0) << '\n'
      << "process " << proc << '\n';
}

void ControlPanelHost::onImGui()
{
    if (!layoutLoaded_) loadLayout();

    if (!visible_) {
        saveLayoutIfChanged();
        return;
    }

    const ImGuiCond cond = fixedLayout_ ? ImGuiCond_Always : ImGuiCond_Once;
    ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), cond);
    ImGui::SetNextWindowSize(ImVec2(430.f, 620.f), cond);
    if (!ImGui::Begin("Control", &visible_)) {
        ImGui::End();
        saveLayoutIfChanged();
        return;
    }

    if (statusDrawer_) {
        statusDrawer_();
        ImGui::Separator();
    }

    ImGui::Text("Page: %s", toString(activePage_));
    ImGui::Separator();

    IEmbeddedPanel* panel = panels_[static_cast<std::size_t>(activePage_)];
    if (!panel) {
        ImGui::TextWrapped("\"%s\" has no panel registered.", toString(activePage_));
    } else {
        ImGui::PushID(toString(activePage_));
        // Let long pages scroll inside the window rather than growing it.
        ImGui::BeginChild("PageBody", ImVec2(0.f, 0.f), false);
        panel->drawContents();
        ImGui::EndChild();
        ImGui::PopID();
    }

    ImGui::End();

    saveLayoutIfChanged();
}

} // namespace VPC
