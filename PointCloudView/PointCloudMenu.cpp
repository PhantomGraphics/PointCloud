#include "PointCloudMenu.h"

#include "ControlPanelHost.h"
#include "ProcessPanel.h"
#include "imgui.h"

namespace VPC {

void PointCloudMenu::init(ControlPanelHost* host, ProcessPanel* processPanel)
{
    host_         = host;
    processPanel_ = processPanel;
}

void PointCloudMenu::pageItem(ControlPage page)
{
    if (!host_) return;
    const bool selected = (host_->getPage() == page) && host_->isVisible();
    if (ImGui::MenuItem(toString(page), nullptr, selected)) {
        host_->setPage(page);
        host_->setVisible(true);
    }
}

void PointCloudMenu::processCategory(ProcessCategory category)
{
    if (!host_ || !processPanel_) return;
    if (!ImGui::BeginMenu(categoryLabel(category))) return;

    const ControlPage processing = ControlPage::Processing;
    for (const auto& item : processesInCategory(category)) {
        const bool selected = (host_->getPage() == processing) &&
                              host_->isVisible() &&
                              (processPanel_->getProcess() == item.id);
        if (ImGui::MenuItem(item.label, nullptr, selected)) {
            processPanel_->setProcess(item.id);
            host_->setPage(processing);
            host_->setVisible(true);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", processName(item.id));
    }
    ImGui::EndMenu();
}

void PointCloudMenu::onImGuiMenuBar()
{
    if (!ImGui::BeginMenu("PointCloud")) return;

    pageItem(ControlPage::Scenes);
    pageItem(ControlPage::Rendering);
    ImGui::Separator();

    processCategory(ProcessCategory::Generate);
    processCategory(ProcessCategory::Features);
    processCategory(ProcessCategory::Filters);
    processCategory(ProcessCategory::Segmentation);
    processCategory(ProcessCategory::Fitting);
    processCategory(ProcessCategory::Registration);
    processCategory(ProcessCategory::Surface);
    ImGui::Separator();

    pageItem(ControlPage::ScenarioBrowser);

    ImGui::EndMenu();
}

} // namespace VPC
