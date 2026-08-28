#include "GSViewMenuBar.h"

#include "../../../CGLib/UIWidgets/FileOpenDialog.h"
#include "imgui.h"

namespace GSView {

void GSViewMenuBar::init(std::function<bool(const std::string&, std::string&)> onOpenFile,
						 std::function<void()> onRequestClose)
{
	onOpenFile_ = std::move(onOpenFile);
	onRequestClose_ = std::move(onRequestClose);
}

void GSViewMenuBar::onImGui()
{
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Open PLY / Splat")) {
				Phantom::UI::FileOpenDialog dlg("Open Gaussian Splat PLY / .splat");
				dlg.addFilter("*.ply");
				dlg.addFilter("*.splat");
				dlg.show();

				const auto path = dlg.getFilePath();
				if (!path.empty() && onOpenFile_) {
					std::string err;
					if (onOpenFile_(path.string(), err)) {
						statusMessage_ = "Opened: " + path.filename().string();
					} else {
						statusMessage_ = "Open failed: " + err;
					}
				}
			}
			if (ImGui::MenuItem("Quit") && onRequestClose_) {
				onRequestClose_();
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}

	if (!statusMessage_.empty()) {
		ImGui::SetNextWindowPos(ImVec2(10.f, 25.f), ImGuiCond_Always);
		ImGui::SetNextWindowBgAlpha(0.35f);
		constexpr ImGuiWindowFlags flags =
			ImGuiWindowFlags_NoDecoration |
			ImGuiWindowFlags_AlwaysAutoResize |
			ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_NoFocusOnAppearing |
			ImGuiWindowFlags_NoNav;
		if (ImGui::Begin("##gsview_status", nullptr, flags)) {
			ImGui::TextUnformatted(statusMessage_.c_str());
		}
		ImGui::End();
	}
}

} // namespace GSView
