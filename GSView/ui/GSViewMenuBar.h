#pragma once

#include <functional>
#include <string>

namespace GSView {

class GSViewMenuBar {
public:
	void init(std::function<bool(const std::string&, std::string&)> onOpenFile,
			  std::function<void()> onRequestClose);

	void onImGui();

private:
	std::function<bool(const std::string&, std::string&)> onOpenFile_;
	std::function<void()> onRequestClose_;

	std::string statusMessage_;
};

} // namespace GSView
