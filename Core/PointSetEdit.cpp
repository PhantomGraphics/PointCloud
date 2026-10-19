#include "PointSetEdit.h"

#include "../../CGLib/ThirdParty/glm/glm/gtc/matrix_transform.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	bool inRange(const PointIndexList& selection, size_t n)
	{
		return selection.empty() || selection.back() < n;   // the lists are sorted
	}

	bool sortedAndRanged(const PointIndexList& selection, size_t n)
	{
		for (size_t i = 0; i < selection.size(); ++i) {
			if (selection[i] >= n) return false;
			if (i > 0 && selection[i] <= selection[i - 1]) return false;
		}
		return true;
	}

	PointSet copyOf(const PointSet& p) { return p; }

}

PointIndexList Phantom::PC::normalizeSelection(PointIndexList indices, size_t pointCount)
{
	std::sort(indices.begin(), indices.end());
	indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
	indices.erase(std::lower_bound(indices.begin(), indices.end(), static_cast<uint32_t>(std::min<size_t>(pointCount, 0xFFFFFFFFu))),
		indices.end());
	return indices;
}

PointIndexList Phantom::PC::combineSelection(const PointIndexList& current, const PointIndexList& hits, SelectMode mode)
{
	PointIndexList out;
	switch (mode) {
	case SelectMode::Replace:
		return hits;
	case SelectMode::Add:
		std::set_union(current.begin(), current.end(), hits.begin(), hits.end(), std::back_inserter(out));
		return out;
	case SelectMode::Remove:
		std::set_difference(current.begin(), current.end(), hits.begin(), hits.end(), std::back_inserter(out));
		return out;
	case SelectMode::Toggle:
		std::set_symmetric_difference(current.begin(), current.end(), hits.begin(), hits.end(), std::back_inserter(out));
		return out;
	}
	return out;
}

PointIndexList Phantom::PC::allIndices(size_t pointCount)
{
	PointIndexList all(pointCount);
	for (size_t i = 0; i < pointCount; ++i) all[i] = static_cast<uint32_t>(i);
	return all;
}

PointIndexList Phantom::PC::invertSelection(const PointIndexList& current, size_t pointCount)
{
	PointIndexList out;
	out.reserve(pointCount > current.size() ? pointCount - current.size() : 0);
	size_t next = 0;
	for (size_t i = 0; i < pointCount; ++i) {
		while (next < current.size() && current[next] < i) ++next;
		if (next < current.size() && current[next] == i) continue;
		out.push_back(static_cast<uint32_t>(i));
	}
	return out;
}

std::optional<Vector3df> Phantom::PC::selectionCentroid(const PointSet& points, const PointIndexList& selection)
{
	if (selection.empty() || !inRange(selection, points.size())) return std::nullopt;
	glm::dvec3 sum(0.0);
	for (const uint32_t i : selection) sum += glm::dvec3(points.positions[i]);
	sum /= static_cast<double>(selection.size());
	return Vector3df(static_cast<float>(sum.x), static_cast<float>(sum.y), static_cast<float>(sum.z));
}

std::optional<Box3df> Phantom::PC::selectionBounds(const PointSet& points, const PointIndexList& selection)
{
	if (selection.empty() || !inRange(selection, points.size())) return std::nullopt;
	auto box = Box3df::createDegeneratedBox();
	for (const uint32_t i : selection) box.add(points.positions[i]);
	return box;
}

glm::mat4 Phantom::PC::translationMatrix(const Vector3df& delta)
{
	return glm::translate(glm::mat4(1.f), delta);
}

glm::mat4 Phantom::PC::rotationAbout(const Vector3df& eulerDeg, const Vector3df& pivot)
{
	glm::mat4 r(1.f);
	r = glm::rotate(r, glm::radians(eulerDeg.y), glm::vec3(0.f, 1.f, 0.f));
	r = glm::rotate(r, glm::radians(eulerDeg.x), glm::vec3(1.f, 0.f, 0.f));
	r = glm::rotate(r, glm::radians(eulerDeg.z), glm::vec3(0.f, 0.f, 1.f));
	return glm::translate(glm::mat4(1.f), pivot) * r * glm::translate(glm::mat4(1.f), -pivot);
}

glm::mat4 Phantom::PC::scaleAbout(const Vector3df& scale, const Vector3df& pivot)
{
	return glm::translate(glm::mat4(1.f), pivot) * glm::scale(glm::mat4(1.f), scale) * glm::translate(glm::mat4(1.f), -pivot);
}

std::optional<PointSet> Phantom::PC::transformSelected(const PointSet& points, const PointIndexList& selection, const glm::mat4& m)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	PointSet out = copyOf(points);
	if (selection.empty()) return out;
	glm::mat3 normalMatrix(1.f);
	if (points.hasNormals()) {
		const glm::mat3 linear(m);
		const float det = glm::determinant(linear);
		if (!(std::abs(det) > 1.0e-20f) || !std::isfinite(det)) return std::nullopt;
		normalMatrix = glm::transpose(glm::inverse(linear));
	}
	for (const uint32_t i : selection) {
		const glm::vec4 p = m * glm::vec4(points.positions[i], 1.f);
		out.positions[i] = Vector3df(p.x, p.y, p.z);
		if (points.hasNormals()) {
			const glm::vec3 n = normalMatrix * points.normals[i];
			const float len = glm::length(n);
			out.normals[i] = len > 0.f ? n / len : points.normals[i];
		}
	}
	return out;
}

std::optional<PointSet> Phantom::PC::deleteSelected(const PointSet& points, const PointIndexList& selection)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	PointSet out;
	const size_t keep = points.size() - selection.size();
	out.positions.reserve(keep);
	out.colors.reserve(keep);
	out.radii.reserve(keep);
	if (points.hasNormals()) out.normals.reserve(keep);
	size_t next = 0;
	for (size_t i = 0; i < points.size(); ++i) {
		if (next < selection.size() && selection[next] == i) { ++next; continue; }
		out.positions.push_back(points.positions[i]);
		out.colors.push_back(points.colors[i]);
		out.radii.push_back(points.radii[i]);
		if (points.hasNormals()) out.normals.push_back(points.normals[i]);
	}
	return out;
}

std::optional<PointSet> Phantom::PC::extractSelected(const PointSet& points, const PointIndexList& selection)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	return subset(points, selection);
}

std::optional<DuplicateResult> Phantom::PC::duplicateSelected(const PointSet& points, const PointIndexList& selection, const Vector3df& offset)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	DuplicateResult r;
	r.points = copyOf(points);
	const uint32_t first = static_cast<uint32_t>(points.size());
	for (size_t k = 0; k < selection.size(); ++k) {
		const uint32_t i = selection[k];
		r.points.positions.push_back(points.positions[i] + offset);
		r.points.colors.push_back(points.colors[i]);
		r.points.radii.push_back(points.radii[i]);
		if (points.hasNormals()) r.points.normals.push_back(points.normals[i]);
		r.newSelection.push_back(first + static_cast<uint32_t>(k));
	}
	return r;
}

std::optional<PointSet> Phantom::PC::setSelectedColor(const PointSet& points, const PointIndexList& selection, const Vector3df& color)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	if (!std::isfinite(color.x) || !std::isfinite(color.y) || !std::isfinite(color.z)) return std::nullopt;
	PointSet out = copyOf(points);
	for (const uint32_t i : selection) out.colors[i] = color;
	return out;
}

std::optional<PointSet> Phantom::PC::setSelectedRadius(const PointSet& points, const PointIndexList& selection, float radius)
{
	if (!sortedAndRanged(selection, points.size())) return std::nullopt;
	if (!std::isfinite(radius) || !(radius > 0.f)) return std::nullopt;
	PointSet out = copyOf(points);
	for (const uint32_t i : selection) out.radii[i] = radius;
	return out;
}
