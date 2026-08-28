#include "GreedyProjectionMeshGenerator.h"

#include "../../CGLib/Space/Space/CompactSpaceHash.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

using namespace Phantom::Math;
using namespace Phantom::Space;
using namespace Phantom::PC;

namespace {
	const float EPS_AREA = 1e-8f;
	const float EPS_DIST = 1e-8f;

	// 三角形が既に存在するか（頂点集合が同じか）を判定する補助
	bool isSameTriangle(const Triangle3df& a, const Triangle3df& b, const float tol = 1e-6f)
	{
		auto va = a.getVertices();
		auto vb = b.getVertices();

		// 各頂点が相手に対応する頂点を持つかチェック（順序無視）
		auto match = [&](const Vector3df& p) {
			for (const auto& q : vb) {
				if (getDistanceSquared(p, q) < tol * tol) return true;
			}
			return false;
		};

		return match(va[0]) && match(va[1]) && match(va[2]);
	}

	// 三角形の有効性（面積）のチェック
	bool isValidTriangle(const Vector3df& p0, const Vector3df& p1, const Vector3df& p2)
	{
		const auto v1 = p1 - p0;
		const auto v2 = p2 - p0;
		const auto cross = glm::cross(v1, v2);
		const auto area2 = glm::dot(cross, cross); // 面積の2乗に比例
		return area2 > EPS_AREA;
	}
}

void GreedyProjectionMeshGenerator::generate(const std::vector<Math::Vector3df>& points)
{
	this->triangles.clear();

	const size_t n = points.size();
	if (n < 3) return;

	// 検索半径を簡易推定: 点群の大きさと点数に基づく経験的値
	// まずは近傍の平均最近傍距離を計算（最初の min(200,n) 点を使って近傍1点を brute-force）
	{
		const size_t sampleCount = std::min<size_t>(n, 200);
		double sumNearest = 0.0;
		for (size_t i = 0; i < sampleCount; ++i) {
			double best = std::numeric_limits<double>::max();
			for (size_t j = 0; j < n; ++j) {
				if (i == j) continue;
				const double d2 = getDistanceSquared(points[i], points[j]);
				if (d2 < best) best = d2;
			}
			if (best < std::numeric_limits<double>::max()) sumNearest += std::sqrt(best);
		}
		const double avgNearest = (sampleCount > 0) ? (sumNearest / static_cast<double>(sampleCount)) : 0.0;
		// 安全マージンを取る
		float searchRadius = static_cast<float>(std::max( avgNearest * 2.0, avgNearest + 1e-6 ));

		// 最低値のガード
		if (searchRadius < 1e-4f) searchRadius = 1e-4f;

		// CompactSpaceHash のテーブルサイズは点数の適当な倍数にする
		const int tableSize = static_cast<int>(std::max<size_t>(16, n * 2 + 1));
		CompactSpaceHash space(searchRadius, tableSize);

		// 全点を追加
		for (const auto& p : points) {
			space.add(p);
		}

		// 各点を中心に局所的な角度ソートを行い貪欲に三角形を追加
		for (size_t i = 0; i < n; ++i) {
			const auto& pi = points[i];
			auto neighbors = space.findNeighborIndices(static_cast<int>(i));
			if (neighbors.size() < 2) continue;

			// 近傍のうち距離が近い順に並べる（中心点から）
			std::sort(neighbors.begin(), neighbors.end(), [&](int a, int b) {
				const float da = getDistanceSquared(points[a], pi);
				const float db = getDistanceSquared(points[b], pi);
				return da < db;
			});

			// 最初の2点で基底を作る（十分に線形でないことを確認）
			int idxA = -1, idxB = -1;
			for (size_t a = 0; a < neighbors.size() && idxA == -1; ++a) {
				for (size_t b = a + 1; b < neighbors.size() && idxA == -1; ++b) {
					const auto& pa = points[neighbors[a]];
					const auto& pb = points[neighbors[b]];
					const auto v1 = pa - pi;
					const auto v2 = pb - pi;
					const auto cross = glm::cross(v1, v2);
					if (glm::dot(cross, cross) > EPS_DIST) {
						idxA = static_cast<int>(neighbors[a]);
						idxB = static_cast<int>(neighbors[b]);
					}
				}
			}
			// 基底が作れなければスキップ
			if (idxA == -1 || idxB == -1) continue;

			// 基底ベクトル u, v と法線 n を作る
			auto u = points[idxA] - pi;
			if (glm::length(u) < 1e-9f) continue;
			u = glm::normalize(u);
			auto nrm = glm::cross(u, points[idxB] - pi);
			if (glm::length(nrm) < 1e-9f) continue;
			nrm = glm::normalize(nrm);
			auto v = glm::cross(nrm, u); // v は u と直交

			// 近傍点を角度でソート
			struct Item { float angle; int index; };
			std::vector<Item> items;
			items.reserve(neighbors.size());
			for (const auto ni : neighbors) {
				if (static_cast<int>(ni) == static_cast<int>(i)) continue;
				const auto vec = points[ni] - pi;
				const float x = glm::dot(vec, u);
				const float y = glm::dot(vec, v);
				const float ang = std::atan2(y, x);
				items.push_back({ ang, ni });
			}
			if (items.size() < 2) continue;
			std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.angle < b.angle; });

			// 角度順に隣接ペアで三角形を生成（閉ループ）
			for (size_t k = 0; k < items.size(); ++k) {
				const int ia = items[k].index;
				const int ib = items[(k + 1) % items.size()].index;

				// 面積が十分であるか
				if (!isValidTriangle(pi, points[ia], points[ib])) continue;

				// 重複チェック（既存三角形と同一頂点集合か）
				Triangle3df tri({ pi, points[ia], points[ib] });

				bool dup = false;
				for (const auto& existing : this->triangles) {
					if (isSameTriangle(existing, tri)) {
						dup = true;
						break;
					}
				}
				if (!dup) {
					this->triangles.push_back(tri);
				}
			}
		}
	}
}