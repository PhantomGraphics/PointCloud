#pragma once

#include <functional>
#include <utility>

namespace VPC {

// A panel whose *content* rendering is separated from its window creation, so
// the same controls can be embedded into the single shared Control window
// managed by ControlPanelHost.
//
// Contract: drawContents() MUST NOT call ImGui::Begin()/End() -- it is always
// invoked with a window already open.
//
// PointCloudView-local for now; promote a minimal slice to CGLib only once the
// PhysicsView and PointCloudView contracts have settled
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md section 4).
class IEmbeddedPanel {
public:
    virtual ~IEmbeddedPanel() = default;
    virtual void drawContents() = 0;
};

// Adapts anything with content-only rendering (e.g. a library panel that
// exposes a drawEmbedded() but cannot derive from IEmbeddedPanel) into an
// IEmbeddedPanel via a callable.
class FnEmbeddedPanel : public IEmbeddedPanel {
public:
    FnEmbeddedPanel() = default;
    explicit FnEmbeddedPanel(std::function<void()> fn) : fn_(std::move(fn)) {}
    void setFunction(std::function<void()> fn) { fn_ = std::move(fn); }
    void drawContents() override { if (fn_) fn_(); }

private:
    std::function<void()> fn_;
};

} // namespace VPC
