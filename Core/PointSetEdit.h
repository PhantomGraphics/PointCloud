#pragma once

#include "PointSet.h"

#include "../../CGLib/Math/Core/glm.h"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Editing of a PointSet by a selection (docs/todo/PLAN_phantomstudio_pointcloud.md Phase 3). A selection is a sorted
		/// list of distinct point indices. Every operation returns a NEW PointSet and leaves its input alone, because a PointSet is
		/// shared and immutable once it is in a scene; the caller installs the result (one undo step) and decides the new selection.
		/// An empty selection changes nothing: the operations return the input's points unchanged.
		using PointIndexList = std::vector<uint32_t>;

		enum class SelectMode { Replace, Add, Remove, Toggle };

		/// @brief Sorted, distinct, in-range (index < pointCount) copy of `indices`.
		PointIndexList normalizeSelection(PointIndexList indices, size_t pointCount);

		/// @brief Applies `hits` to `current` (both normalized) the way a click or rectangle does: Replace = hits, Add = union,
		/// Remove = difference, Toggle = symmetric difference.
		PointIndexList combineSelection(const PointIndexList& current, const PointIndexList& hits, SelectMode mode);

		/// All of 0..pointCount-1.
		PointIndexList allIndices(size_t pointCount);

		/// The indices of 0..pointCount-1 that are not in `current`.
		PointIndexList invertSelection(const PointIndexList& current, size_t pointCount);

		/// @brief Mean of the selected centres. std::nullopt when the selection is empty or has an out-of-range index.
		std::optional<Math::Vector3df> selectionCentroid(const PointSet& points, const PointIndexList& selection);

		/// @brief Bounds of the selected centres (no radius). std::nullopt when the selection is empty or out of range.
		std::optional<Math::Box3df> selectionBounds(const PointSet& points, const PointIndexList& selection);

		/// Local-space transforms, in the same convention as an object's transform: Translate, then rotation Ry * Rx * Rz (degrees),
		/// then scale. `rotationAbout` / `scaleAbout` act about `pivot`.
		glm::mat4 translationMatrix(const Math::Vector3df& delta);
		glm::mat4 rotationAbout(const Math::Vector3df& eulerDeg, const Math::Vector3df& pivot);
		glm::mat4 scaleAbout(const Math::Vector3df& scale, const Math::Vector3df& pivot);

		/// @brief The points with the selected ones moved by `m`. Normals follow by the inverse transpose (renormalized). Radii are
		/// kept: a transform moves points, it does not resize them (use setRadius for that). Returns std::nullopt when `selection` has an
		/// out-of-range index or `m` is singular for a point with normals (a zero scale).
		std::optional<PointSet> transformSelected(const PointSet& points, const PointIndexList& selection, const glm::mat4& m);

		/// @brief The points without the selected ones (the order of the rest is kept). std::nullopt on an out-of-range index.
		std::optional<PointSet> deleteSelected(const PointSet& points, const PointIndexList& selection);

		/// @brief The selected points alone (the order is kept). std::nullopt on an out-of-range index.
		std::optional<PointSet> extractSelected(const PointSet& points, const PointIndexList& selection);

		struct DuplicateResult
		{
			PointSet points;                 ///< the original points followed by one copy of each selected point
			PointIndexList newSelection;     ///< the indices of the copies
		};

		/// @brief Appends a copy of every selected point, moved by `offset`. std::nullopt on an out-of-range index.
		std::optional<DuplicateResult> duplicateSelected(const PointSet& points, const PointIndexList& selection,
			const Math::Vector3df& offset);

		/// Colour (linear RGB, finite) / radius (finite, > 0) of the selected points. std::nullopt on an out-of-range index or an
		/// invalid value.
		std::optional<PointSet> setSelectedColor(const PointSet& points, const PointIndexList& selection, const Math::Vector3df& color);
		std::optional<PointSet> setSelectedRadius(const PointSet& points, const PointIndexList& selection, float radius);

	}
}
