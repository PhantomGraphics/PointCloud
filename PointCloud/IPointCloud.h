#pragma once

#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Box3d.h"
#include <cstddef>
#include <vector>

namespace Phantom
{
	namespace PC
	{
		/// @brief Common read/write interface for point cloud containers.
		///
		/// Lets generic code (algorithms, tests, future point types) work with "any point
		/// cloud" via a common base rather than a specific concrete type. Each implementer
		/// (PointCloud, ColoredPointCloud, ...) keeps its own SoA (Structure of Arrays)
		/// storage internally - this interface only standardizes access to position/size,
		/// so it does not bring back the per-point heap allocation the AoS->SoA migration removed.
		class IPointCloud
		{
		public:
			virtual ~IPointCloud() = default;

			/// @brief Returns the number of points.
			virtual size_t size() const = 0;

			/// @brief Returns the position of the point at the given index.
			/// @param index The index of the point (must be < size()).
			virtual Math::Vector3df getPosition(size_t index) const = 0;

			/// @brief Returns a const reference to the underlying position array.
			virtual const std::vector<Math::Vector3df>& getPositions() const = 0;

			/// @brief Replaces the entire position array.
			/// @param positions The new positions (moved in).
			virtual void setPositions(std::vector<Math::Vector3df> positions) = 0;

			/// @brief Appends a single point.
			/// @param position The position of the point to add.
			virtual void addPosition(const Math::Vector3df& position) = 0;

			/// @brief Returns true if the point cloud has no points.
			bool empty() const { return size() == 0; }

			/// @brief Computes the axis-aligned bounding box from getPosition()/size().
			/// Implemented once here (in terms of the interface above) so each concrete
			/// point cloud type doesn't need to repeat it.
			Math::Box3df getBoundingBox() const {
				auto bb = Math::Box3d<float>::createDegeneratedBox();
				const size_t n = size();
				for (size_t i = 0; i < n; ++i) bb.add(getPosition(i));
				return bb;
			}
		};

		/// @brief Appends every position in `cloud` to `consumer` via
		/// `consumer.add(const Math::Vector3df&)`. Lets existing single-point `add()`-style
		/// algorithms (NormalEstimator, DownSampler, CurvatureEstimator, DensityEstimator,
		/// DensityBasedFilter, SORFilter, RadiusOutlierFilter, PassThroughFilter, MLSSurface,
		/// ...) consume any IPointCloud without each one needing an IPointCloud-specific
		/// overload.
		template <typename PositionConsumer>
		void feedPositions(PositionConsumer& consumer, const IPointCloud& cloud) {
			const size_t n = cloud.size();
			for (size_t i = 0; i < n; ++i) {
				consumer.add(cloud.getPosition(i));
			}
		}
	}
}
