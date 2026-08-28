#pragma once

#include "CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Classifies point cloud points as ground/non-ground using a simplified
		/// Progressive Morphological Filter (PMF; Zhang et al. 2003), a common preprocessing
		/// step for LiDAR/drone scan data before further processing (e.g. isolating buildings,
		/// vegetation, or objects sitting on terrain).
		class GroundExtractor
		{
		public:
			GroundExtractor() = default;
			~GroundExtractor() = default;

			/// @brief Adds a point to be classified.
			void add(const Math::Vector3df& position) { positions.push_back(position); }

			/// @brief Tuning parameters for the progressive morphological filter.
			struct Params
			{
				float cellSize = 1.0f;                   ///< Grid cell size (XY), in world units.
				float initialWindowSize = 1.0f;          ///< Initial morphological window half-width, in cells.
				float maxWindowSize = 16.0f;              ///< Window growth stops once it exceeds this (in cells).
				float windowGrowthFactor = 2.0f;          ///< Window half-width is multiplied by this each step (must be > 1).
				float slope = 0.3f;                       ///< Terrain slope parameter: how fast the elevation threshold grows with window size.
				float initialElevationThreshold = 0.2f;   ///< dh0: elevation-difference threshold at the smallest window.
				float maxElevationThreshold = 3.0f;       ///< dhMax: elevation-difference threshold is capped at this.
				float finalElevationThreshold = 0.3f;     ///< Final per-point tolerance above the filtered ground surface.
			};

			/// @brief Classifies each added point as ground/non-ground. Rasterizes points into an
			/// XY grid keyed by minimum elevation, then repeatedly grayscale-opens that grid
			/// (erosion then dilation) with growing window sizes; a grid cell whose elevation
			/// drops by more than a (window-size-dependent) threshold after opening is flagged
			/// non-ground going forward -- this removes objects (buildings/vegetation/vehicles)
			/// standing on the terrain while preserving gentle slope. A point is finally
			/// classified ground if its cell was never flagged non-ground AND its own elevation
			/// is within `finalElevationThreshold` of the fully-filtered surface at that cell.
			/// @return false if no points were added, or `params` has cellSize <= 0,
			///         initialWindowSize <= 0, or windowGrowthFactor <= 1.
			bool extract(const Params& params);

			/// @brief Classifies each added point as ground/non-ground using default Params.
			bool extract();

			/// @brief Returns the ground/non-ground flags, same order as added points.
			std::vector<bool> getGroundFlags() const { return groundFlags; }

		private:
			std::vector<Math::Vector3df> positions;
			std::vector<bool> groundFlags;

			/// @brief Applies a square-window min filter (useMin=true, erosion) or max filter
			/// (useMin=false, dilation) to a width*height grid. Cells holding the sentinel
			/// "no data" value (+infinity) are excluded from the window computation; a cell whose
			/// entire window is "no data" stays "no data".
			static std::vector<float> morphologyFilter(const std::vector<float>& grid, int width, int height, int radius, bool useMin);
		};

		// Defined out-of-line: Params() needs GroundExtractor (the enclosing class) to be
		// complete before its nested Params' default member initializers can be used
		// (MSVC is lenient about this; clang/GCC require strict standard conformance).
		inline bool GroundExtractor::extract() { return extract(Params()); }

	}
}
