#pragma once

#include "CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Segments a point cloud into smooth surface patches via normal/curvature-based
		/// region growing (Rabbani et al. 2006 / PCL's RegionGrowing): starting from the
		/// smoothest (lowest-curvature) unvisited point, repeatedly absorbs k-NN neighbors whose
		/// normal stays within a smoothness angle threshold of the current point, using each
		/// absorbed point's own curvature to decide whether growth continues from it. Separates
		/// cases DBSCAN/Euclidean clustering can't: multiple coplanar-but-disjoint-in-density
		/// surfaces, or a single dense cluster spanning a sharp fold (rejected here by the normal
		/// angle test even though it's spatially/density-wise contiguous).
		class RegionGrowing
		{
		public:
			RegionGrowing() = default;
			~RegionGrowing() = default;

			/// @brief Adds a point with its precomputed normal and curvature (e.g. from
			/// NormalEstimator/CurvatureEstimator).
			void add(const Math::Vector3df& position, const Math::Vector3df& normal, double curvature) {
				positions.push_back(position);
				normals.push_back(normal);
				curvatures.push_back(curvature);
			}

			struct Params
			{
				size_t kNeighbors = 30;                    ///< Number of nearest neighbors examined per growth step.
				float smoothnessThresholdRad = 0.0872665f; ///< Max angle (radians, ~5 deg) between normals to absorb a neighbor.
				double curvatureThreshold = 1.0;            ///< Absorbed points with curvature below this also become growth seeds.
				size_t minClusterSize = 10;                 ///< Regions smaller than this are discarded (left unclustered).
			};

			/// @brief Runs region growing over all added points.
			/// @return false if there are no points, or positions/normals/curvatures sizes mismatch.
			bool segment(const Params& params);

			/// @brief Runs region growing over all added points using default Params.
			bool segment();

			/// @brief Returns each point's cluster label, same order as added points: -1 if
			/// unclustered (part of no region reaching minClusterSize), else a 0-based cluster index.
			std::vector<int> getLabels() const { return labels; }

			/// @brief Returns the number of clusters found (0-based labels range [0, count-1]).
			size_t getClusterCount() const { return clusterCount; }

		private:
			std::vector<Math::Vector3df> positions;
			std::vector<Math::Vector3df> normals;
			std::vector<double> curvatures;
			std::vector<int> labels;
			size_t clusterCount = 0;
		};

		// Defined out-of-line: Params() needs RegionGrowing (the enclosing class) to be
		// complete before its nested Params' default member initializers can be used
		// (MSVC is lenient about this; clang/GCC require strict standard conformance).
		inline bool RegionGrowing::segment() { return segment(Params()); }

	}
}
