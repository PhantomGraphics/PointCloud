#pragma once

#include "../../CGLib/Math/Core/Box3d.h"
#include "../../CGLib/Math/Core/Vector3d.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief A coloured point set with a radius per point, as used for point clouds treated as
		/// scene material (docs/todo/PLAN_phantomstudio_pointcloud.md).
		///
		/// Every attribute is a parallel array. Positions and radii are in the local space of the
		/// owning object (a radius is a length, so a non-uniform scale turns a point into an
		/// ellipsoid). Colours are LINEAR RGB; the PLY import/export converts from/to sRGB. Normals
		/// are optional (empty = none) and are only carried for round trips; a point is rendered as
		/// a sphere.
		///
		/// A PointSet is treated as immutable once it is shared (SharedPointSet). Placement, material
		/// changes and object duplication share the pointer; an edit builds a new PointSet.
		struct PointSet
		{
			std::vector<Math::Vector3df> positions;
			std::vector<Math::Vector3df> colors;   ///< linear RGB, same size as positions
			std::vector<float>           radii;    ///< local-space length > 0, same size as positions
			std::vector<Math::Vector3df> normals;  ///< empty, or same size as positions

			size_t size() const { return positions.size(); }
			bool   empty() const { return positions.empty(); }
			bool   hasNormals() const { return !normals.empty(); }

			/// @brief Checks the invariants: equal array sizes, finite positions/colours/normals,
			/// finite radii > 0.
			/// @return An empty string when valid, otherwise a diagnostic naming the first problem.
			std::string validate() const;

			/// @brief Bounds of the centres. A degenerated (inverted) box when empty.
			Math::Box3df bounds() const;

			/// @brief Bounds of the spheres (centre +- radius on every axis). Degenerated when empty.
			Math::Box3df boundsWithRadius() const;

			/// @brief Heap bytes held by the arrays.
			size_t memoryBytes() const;
		};

		/// Shared, never mutated through this pointer.
		using SharedPointSet = std::shared_ptr<const PointSet>;

		/// @brief Moves `points` into a shared immutable set after validating it.
		/// @return nullptr and a diagnostic in `error` when it does not validate.
		SharedPointSet makeSharedPointSet(PointSet&& points, std::string* error = nullptr);

		/// @brief The points at `indices`, in that order (duplicates allowed).
		/// @return std::nullopt when an index is out of range.
		std::optional<PointSet> subset(const PointSet& source, const std::vector<uint32_t>& indices);

		/// @brief A radius for a set without one: half the mean spacing of `count` points spread
		/// through `bounds` (the cube root of the volume per point), never below 1e-6.
		float defaultRadiusForBounds(const Math::Box3df& bounds, size_t count);

		/// @brief FNV-1a 64 of the arrays' bytes (positions, colours, radii, normals). Equal sets hash
		/// equal; it names and verifies the binary asset.
		uint64_t contentHash(const PointSet& points);

		/// sRGB transfer functions on one channel in [0,1] (input is clamped).
		float srgbToLinear(float v);
		float linearToSrgb(float v);

	}
}
