#pragma once

#include "World.h"
#include <functional>

namespace VPC {

/// @brief Base interface for process-view panels (algorithm UI panels).
///
/// Each algorithm's ImGui panel implements this interface.
/// PointCloudApp holds the active panel as std::unique_ptr<IProcessView>.
class IProcessView {
public:
    virtual ~IProcessView() = default;

    virtual const char* getName() const = 0;

    /// @brief Render ImGui widgets and execute processing.
    /// @param world         Scene management object.
    /// @param activeSceneId Currently selected scene ID.
    /// @param onRebuild     Callback to trigger GPU vertex buffer rebuild.
    virtual void onImGui(World& world, int activeSceneId,
                         const std::function<void()>& onRebuild) = 0;
};

} // namespace VPC
