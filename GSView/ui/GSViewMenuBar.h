#pragma once

#include <functional>
#include <string>

namespace GSView {

class GSViewMenuBar {
public:
	void init(std::function<bool(const std::string&, std::string&)> onOpenFile,
			  std::function<void()> onRequestClose);

	// Extra menus drawn inside the main menu bar (e.g. the shell's View menu).
	void setExtraMenus(std::function<void()> f) { extraMenus_ = std::move(f); }

	void onImGui();

private:
	std::function<bool(const std::string&, std::string&)> onOpenFile_;
	std::function<void()> onRequestClose_;
	std::function<void()> extraMenus_;

	std::string statusMessage_;
};

} // namespace GSView
