#include "GroundExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	constexpr float kInvalid = std::numeric_limits<float>::infinity();
}

std::vector<float> GroundExtractor::morphologyFilter(const std::vector<float>& grid, int width, int height, int radius, bool useMin)
{
	std::vector<float> result(grid.size(), kInvalid);
	// MSVC's OpenMP 2.0 support has no collapse() clause, so only the outer (y) loop is
	// parallelized; each row is still O(width * (2*radius+1)^2) of independent work.
#pragma omp parallel for
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			float best = useMin ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity();
			bool found = false;
			for (int dy = -radius; dy <= radius; ++dy) {
				const int ny = y + dy;
				if (ny < 0 || ny >= height) continue;
				for (int dx = -radius; dx <= radius; ++dx) {
					const int nx = x + dx;
					if (nx < 0 || nx >= width) continue;
					const float v = grid[static_cast<size_t>(ny) * width + nx];
					if (!std::isfinite(v)) continue;
					found = true;
					best = useMin ? std::min(best, v) : std::max(best, v);
				}
			}
			result[static_cast<size_t>(y) * width + x] = found ? best : kInvalid;
		}
	}
	return result;
}

bool GroundExtractor::extract(const Params& params)
{
	groundFlags.clear();

	if (positions.empty()
		|| params.cellSize <= 0.0f
		|| params.initialWindowSize <= 0.0f
		|| params.windowGrowthFactor <= 1.0f) {
		return false;
	}

	float minX = positions[0].x, maxX = positions[0].x;
	float minY = positions[0].y, maxY = positions[0].y;
	for (const auto& p : positions) {
		minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
		minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
	}

	const int width = static_cast<int>(std::floor((maxX - minX) / params.cellSize)) + 1;
	const int height = static_cast<int>(std::floor((maxY - minY) / params.cellSize)) + 1;

	std::vector<float> minZGrid(static_cast<size_t>(width) * height, kInvalid);
	std::vector<size_t> cellIndexOfPoint(positions.size());

	for (size_t i = 0; i < positions.size(); ++i) {
		const auto& p = positions[i];
		const int ix = std::min(width - 1, static_cast<int>((p.x - minX) / params.cellSize));
		const int iy = std::min(height - 1, static_cast<int>((p.y - minY) / params.cellSize));
		const size_t cell = static_cast<size_t>(iy) * width + ix;
		cellIndexOfPoint[i] = cell;
		minZGrid[cell] = std::min(minZGrid[cell], p.z);
	}

	// Working elevation surface, progressively opened with growing windows; cells flagged
	// non-ground stay flagged (monotonic) even if a later, larger window would no longer trip
	// the threshold there.
	std::vector<float> surface = minZGrid;
	std::vector<char> isGroundCell(surface.size(), 1);

	for (float w = params.initialWindowSize; w <= params.maxWindowSize; w *= params.windowGrowthFactor) {
		const int radius = std::max(1, static_cast<int>(std::lround(w)));

		const auto eroded = morphologyFilter(surface, width, height, radius, true);
		const auto opened = morphologyFilter(eroded, width, height, radius, false);

		const float threshold = std::min(params.maxElevationThreshold,
			params.initialElevationThreshold + params.slope * static_cast<float>(radius) * params.cellSize);

		for (size_t c = 0; c < surface.size(); ++c) {
			if (std::isfinite(surface[c]) && std::isfinite(opened[c])
				&& (surface[c] - opened[c] > threshold)) {
				isGroundCell[c] = 0;
			}
		}
		surface = opened;
	}

	groundFlags.resize(positions.size());
	for (size_t i = 0; i < positions.size(); ++i) {
		const size_t cell = cellIndexOfPoint[i];
		const float heightAboveSurface = positions[i].z - surface[cell];
		groundFlags[i] = (isGroundCell[cell] != 0) && (heightAboveSurface <= params.finalElevationThreshold);
	}

	return true;
}
