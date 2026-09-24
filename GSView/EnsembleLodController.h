#pragma once

// EnsembleLodController moved to CGLib/Graphics/EnsembleLodController.h
// (header-only, Phantom::Graphics) so PhysicsView's Flame PBVR can reuse it
// (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 4). This alias keeps
// GSView's and PointCloudTest's existing `GSView::EnsembleLodController`
// spelling working unchanged.

#include "../../CGLib/Graphics/EnsembleLodController.h"

namespace GSView {
using EnsembleLodController = Phantom::Graphics::EnsembleLodController;
}
