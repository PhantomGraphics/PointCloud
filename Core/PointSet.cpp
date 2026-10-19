#include "PointSet.h"

#include <algorithm>
#include <cmath>

using namespace Phantom::PC;
using namespace Phantom::Math;

namespace {

	bool finite(const Vector3df& v)
	{
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	}

	uint64_t fnv(uint64_t h, const void* data, size_t bytes)
	{
		const unsigned char* p = static_cast<const unsigned char*>(data);
		for (size_t i = 0; i < bytes; ++i) {
			h ^= p[i];
			h *= 1099511628211ull;
		}
		return h;
	}

}

std::string PointSet::validate() const
{
	const size_t n = positions.size();
	if (colors.size() != n)
		return "color count " + std::to_string(colors.size()) + " does not match point count " + std::to_string(n);
	if (radii.size() != n)
		return "radius count " + std::to_string(radii.size()) + " does not match point count " + std::to_string(n);
	if (!normals.empty() && normals.size() != n)
		return "normal count " + std::to_string(normals.size()) + " does not match point count " + std::to_string(n);
	for (size_t i = 0; i < n; ++i) {
		if (!finite(positions[i])) return "point " + std::to_string(i) + " has a non-finite coordinate";
		if (!finite(colors[i])) return "point " + std::to_string(i) + " has a non-finite color";
		if (!std::isfinite(radii[i]) || !(radii[i] > 0.0f))
			return "point " + std::to_string(i) + " has an invalid radius (must be finite and > 0)";
		if (!normals.empty() && !finite(normals[i])) return "point " + std::to_string(i) + " has a non-finite normal";
	}
	return {};
}

Box3df PointSet::bounds() const
{
	auto box = Box3df::createDegeneratedBox();
	for (const auto& p : positions) box.add(p);
	return box;
}

Box3df PointSet::boundsWithRadius(float radiusScale) const
{
	auto box = Box3df::createDegeneratedBox();
	for (size_t i = 0; i < positions.size(); ++i) {
		const auto& p = positions[i];
		const float r = i < radii.size() ? radii[i] * radiusScale : 0.0f;
		box.add(Vector3df(p.x - r, p.y - r, p.z - r));
		box.add(Vector3df(p.x + r, p.y + r, p.z + r));
	}
	return box;
}

size_t PointSet::memoryBytes() const
{
	return (positions.capacity() + colors.capacity() + normals.capacity()) * sizeof(Vector3df)
		+ radii.capacity() * sizeof(float);
}

SharedPointSet Phantom::PC::makeSharedPointSet(PointSet&& points, std::string* error)
{
	const std::string problem = points.validate();
	if (!problem.empty()) {
		if (error) *error = problem;
		return nullptr;
	}
	return std::make_shared<const PointSet>(std::move(points));
}

std::optional<PointSet> Phantom::PC::subset(const PointSet& source, const std::vector<uint32_t>& indices)
{
	PointSet out;
	out.positions.reserve(indices.size());
	out.colors.reserve(indices.size());
	out.radii.reserve(indices.size());
	if (source.hasNormals()) out.normals.reserve(indices.size());
	for (const uint32_t i : indices) {
		if (i >= source.size()) return std::nullopt;
		out.positions.push_back(source.positions[i]);
		out.colors.push_back(source.colors[i]);
		out.radii.push_back(source.radii[i]);
		if (source.hasNormals()) out.normals.push_back(source.normals[i]);
	}
	return out;
}

float Phantom::PC::defaultRadiusForBounds(const Box3df& bounds, size_t count)
{
	constexpr float kMinRadius = 1.0e-6f;
	if (count == 0) return 0.01f;
	const Vector3df len = bounds.getLength();
	if (!std::isfinite(len.x) || !std::isfinite(len.y) || !std::isfinite(len.z)) return 0.01f;
	// Planar or linear scans have a near-zero extent on some axes; floor each axis at 1% of the
	// largest so the radius reflects the cloud's size, not its thinnest dimension.
	const float maxLen = std::max({ len.x, len.y, len.z });
	if (!(maxLen > 0.0f)) return 0.01f;
	const float floorLen = 0.01f * maxLen;
	const float volume = std::max(len.x, floorLen) * std::max(len.y, floorLen) * std::max(len.z, floorLen);
	const float spacing = std::cbrt(volume / static_cast<float>(count));
	return std::max(0.5f * spacing, kMinRadius);
}

uint64_t Phantom::PC::contentHash(const PointSet& points)
{
	uint64_t h = 14695981039346656037ull;
	auto feed = [&](const auto& v) { if (!v.empty()) h = fnv(h, v.data(), v.size() * sizeof(v[0])); };
	feed(points.positions);
	feed(points.colors);
	feed(points.radii);
	feed(points.normals);
	return h;
}

float Phantom::PC::srgbToLinear(float v)
{
	v = std::clamp(v, 0.0f, 1.0f);
	return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
}

float Phantom::PC::linearToSrgb(float v)
{
	v = std::clamp(v, 0.0f, 1.0f);
	return v <= 0.0031308f ? v * 12.92f : 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;
}
