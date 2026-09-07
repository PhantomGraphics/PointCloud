#pragma once

#include "IEmbeddedPanel.h"

#include <array>
#include <functional>
#include <string>

namespace VPC {

class PointCloudRenderer;

// The "Import / Export" page. Path input + file dialogs + last-result text.
// The actual I/O is delegated to the PointCloudApp load/save callbacks; this
// panel only builds the request and reports the outcome
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 4).
class ImportExportPanel : public IEmbeddedPanel {
public:
    using IoFn = std::function<bool(const std::string&, std::string&)>;

    void init(PointCloudRenderer* renderer, IoFn onLoad, IoFn onSave);

    void drawContents() override;

    const std::string& lastStatus() const { return statusMessage_; }

private:
    PointCloudRenderer* renderer_ = nullptr;
    IoFn onLoad_;
    IoFn onSave_;

    std::array<char, 512> importPath_{};
    std::array<char, 512> exportPath_{};
    std::string statusMessage_;
};

} // namespace VPC
