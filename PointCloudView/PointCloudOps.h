#pragma once

#include <string>

namespace VPC {

class World;

// Typed point-cloud processing functions shared by the GUI panels and the
// scenario CommandDispatcher, so both entry points produce byte-identical
// geometry, colour, visibility and selection
// (docs/todo/PLAN_pointcloudview_gui_restructuring.md Phase 4). Migrated one
// operation at a time; DownSample is the first.
namespace ops {

struct ProcessOutcome {
    bool        ok = false;
    std::string message;       // human-readable; on failure, the reason (no "Error:" prefix)
    int         resultSceneId = -1;
};

struct DownSampleParams {
    float cellSize = 0.05f;
};

// Voxel-grid downsample of scene `activeSceneId` in `world`. On success adds a
// "DownSampleResult" scene (per-point colour = kept-point ratio, grayscale),
// hides the source scene, and reports the new scene id. Does NOT itself change
// the active selection or rebuild -- the caller applies ProcessOutcome.
ProcessOutcome downSample(World& world, int activeSceneId, const DownSampleParams& p);

} // namespace ops
} // namespace VPC
