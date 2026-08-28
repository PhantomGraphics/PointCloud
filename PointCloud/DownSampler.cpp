#include "DownSampler.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

using namespace Phantom::Math;
using namespace Phantom::PC;

namespace {
	// Integer grid-cell coordinate, i.e. floor(position / voxelSize) per axis.
	struct VoxelIndex {
		int64_t x, y, z;
		bool operator==(const VoxelIndex& other) const {
			return x == other.x && y == other.y && z == other.z;
		}
	};

	struct VoxelIndexHash {
		size_t operator()(const VoxelIndex& v) const {
			// boost::hash_combine-style mixing of the three axis hashes.
			size_t seed = std::hash<int64_t>{}(v.x);
			seed ^= std::hash<int64_t>{}(v.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
			seed ^= std::hash<int64_t>{}(v.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
			return seed;
		}
	};

	VoxelIndex toVoxelIndex(const Vector3df& p, double invVoxelSize) {
		return VoxelIndex{
			static_cast<int64_t>(std::floor(static_cast<double>(p.x) * invVoxelSize)),
			static_cast<int64_t>(std::floor(static_cast<double>(p.y) * invVoxelSize)),
			static_cast<int64_t>(std::floor(static_cast<double>(p.z) * invVoxelSize))
		};
	}

	struct VoxelAccumulator {
		Vector3df sum{ 0.0f, 0.0f, 0.0f };
		int count = 0;
	};
}

void DownSampler::execute(const double voxelSize)
{
	downSampled.clear();
	if (pointCloud.empty()) return;

	if (voxelSize <= 0.0) {
		// Degenerate cell size: every point is its own voxel (pass-through) rather than
		// dividing by zero below.
		downSampled = pointCloud;
		return;
	}

	// Standard voxel-grid downsampling (as in PCL's VoxelGrid / Open3D's voxel_down_sample):
	// bucket every point into the fixed integer grid cell it falls in (floor(position /
	// voxelSize) per axis), then replace each occupied cell with the centroid of the points
	// inside it. This partitions space into a grid that's independent of point order or which
	// point happens to be visited first -- every point maps to exactly one voxel purely from its
	// own coordinates, unlike a radius-based nearest-neighbor search.
	//
	// Deliberately not OpenMP-parallelized: per-point work here is just a handful of floor()s
	// plus one hash-map lookup, so a per-thread-map-then-merge parallelization was measured
	// (via PointCloud/benchmark_open3d/) to be 2-3x SLOWER than this single-threaded version at
	// every tested size up to 1,000,000 points -- thread fork/join, per-thread map allocation,
	// and the serial merge pass all cost more than the parallel section saves. Revisit only if a
	// fundamentally different strategy (e.g. parallel sort-based bucketing) is worth the added
	// complexity for a demonstrated real workload.
	const double invVoxelSize = 1.0 / voxelSize;

	std::unordered_map<VoxelIndex, VoxelAccumulator, VoxelIndexHash> voxels;
	voxels.reserve(pointCloud.size());

	// Records each voxel's first-occurrence order so the output order matches the order points
	// were added in.
	std::vector<VoxelIndex> voxelOrder;
	voxelOrder.reserve(pointCloud.size());

	for (const auto& p : pointCloud) {
		const auto index = ::toVoxelIndex(p, invVoxelSize);
		const auto it = voxels.find(index);
		if (it == voxels.end()) {
			voxelOrder.push_back(index);
			voxels.emplace(index, VoxelAccumulator{ p, 1 });
		} else {
			it->second.sum += p;
			++it->second.count;
		}
	}

	downSampled.reserve(voxelOrder.size());
	for (const auto& index : voxelOrder) {
		const auto& acc = voxels[index];
		downSampled.push_back(acc.sum / static_cast<float>(acc.count));
	}
}
