// PoissonSurface.hpp
#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <numeric>
#include <cassert>

/// @brief Input point with position and normal for Poisson surface reconstruction.
/// Normals must be consistently oriented across the point cloud.
struct PSPoint {
    float x, y, z;    ///< 3D position of the point.
    float nx, ny, nz; ///< Normal vector (must be consistently oriented).
};

/// @brief Output triangle mesh produced by Poisson surface reconstruction.
struct PSTriangleMesh {
    std::vector<std::array<float, 3>>    vertices; ///< Vertex positions; each element is {x, y, z}.
    std::vector<std::array<uint32_t, 3>> faces;    ///< Triangle faces; each element holds three vertex indices.
};

/// @brief Poisson surface reconstruction from an oriented point cloud.
/// Solves the Poisson equation on a voxel grid to obtain an implicit function,
/// then extracts the iso-surface as a triangle mesh using a tetrahedral marching approach.
class PoissonSurface {
public:
    /// @brief Configuration parameters for the reconstruction.
    struct Config {
        int   resolution = 128;      ///< Voxel grid resolution per side (64-256 recommended).
        float samplesPerCell = 1.5f; ///< Influence radius of normal splatting (in cell units).
        float isoLevel = 0.0f;       ///< Iso-surface level value (0 recommended).
        int   maxIters = 200;        ///< Maximum SOR solver iterations.
        float omega = 1.8f;          ///< SOR relaxation factor (1.7-1.95 is stable).
        float bboxPadding = 2.5f;    ///< AABB padding amount (in cell units).
        bool  clampToBBox = true;    ///< Treat grid boundaries as Neumann condition (zero normal derivative).
    };

    /// @brief Constructs a PoissonSurface with default configuration.
    explicit PoissonSurface();

    /// @brief Constructs a PoissonSurface with the given configuration.
    /// @param cfg Reconstruction parameters.
    explicit PoissonSurface(const Config& cfg) : cfg_(cfg) {}

    /// @brief Reconstructs a triangle mesh from the given oriented point cloud.
    /// @param points Input points with normals. Must not be empty.
    /// @return The reconstructed triangle mesh.
    PSTriangleMesh reconstruct(const std::vector<PSPoint>& points) {
        if (points.empty()) {
            return {};
        }
        computeBounds(points);
        allocateGrid();
        splatNormals(points);
        buildDivergence();
        solvePoisson();
        PSTriangleMesh mesh;
        extractIsoSurface(mesh);
        return mesh;
    }

private:
    struct Bounds {
        std::array<float, 3> min, max;
        std::array<float, 3> size;
        float cell;
        std::array<int, 3> dim;
    } b_;

    Config cfg_;

    // Scalar field u, right-hand side f (= div V), and normal vector field V (at grid nodes).
    std::vector<float> u_, f_;
    std::vector<std::array<float, 3>> V_;

    inline int idx(int i, int j, int k) const {
        return (k * b_.dim[1] + j) * b_.dim[0] + i;
    }

    inline bool inside(int i, int j, int k) const {
        return (i >= 0 && j >= 0 && k >= 0 && i < b_.dim[0] && j < b_.dim[1] && k < b_.dim[2]);
    }

    void computeBounds(const std::vector<PSPoint>& pts) {
        auto mi = std::array<float, 3>{ std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max() };
        auto ma = std::array<float, 3>{ -std::numeric_limits<float>::max(),
                                       -std::numeric_limits<float>::max(),
                                       -std::numeric_limits<float>::max() };
        for (auto& p : pts) {
            mi[0] = std::min(mi[0], p.x); mi[1] = std::min(mi[1], p.y); mi[2] = std::min(mi[2], p.z);
            ma[0] = std::max(ma[0], p.x); ma[1] = std::max(ma[1], p.y); ma[2] = std::max(ma[2], p.z);
        }
        // Expand to a cube and add padding.
        std::array<float, 3> sz{ ma[0] - mi[0], ma[1] - mi[1], ma[2] - mi[2] };
        float maxSz = std::max({ sz[0], sz[1], sz[2] });
        if (maxSz <= 0) maxSz = 1.0f;

        // Compute cell size from resolution.
        int n = std::max(8, cfg_.resolution);
        float cell = maxSz / (n - 1 - 2 * cfg_.bboxPadding);
        if (cell <= 0) cell = 1.0f;

        // Center the grid and apply padding.
        b_.cell = cell;
        b_.size = { cell * (n - 1), cell * (n - 1), cell * (n - 1) };
        b_.min = { (mi[0] + ma[0]) * 0.5f - b_.size[0] * 0.5f,
                   (mi[1] + ma[1]) * 0.5f - b_.size[1] * 0.5f,
                   (mi[2] + ma[2]) * 0.5f - b_.size[2] * 0.5f };
        b_.max = { b_.min[0] + b_.size[0], b_.min[1] + b_.size[1], b_.min[2] + b_.size[2] };
        b_.dim = { n, n, n };
    }

    void allocateGrid() {
        size_t N = size_t(b_.dim[0]) * b_.dim[1] * b_.dim[2];
        u_.assign(N, 0.0f);
        f_.assign(N, 0.0f);
        V_.assign(N, { 0.0f,0.0f,0.0f });
    }

    // Splat each point's normal onto surrounding grid nodes with a Gaussian weight to build V(x).
    void splatNormals(const std::vector<PSPoint>& pts) {
        std::vector<float> W(V_.size(), 0.0f);
        const float r = cfg_.samplesPerCell * b_.cell;
        const float r2 = r * r;
        const float inv2sig2 = 1.0f / (2.0f * r2);

        for (const auto& p : pts) {
            std::array<float, 3> g = worldToGrid({ p.x, p.y, p.z });
            int i0 = std::max(0, int(std::floor(g[0] - cfg_.samplesPerCell)));
            int j0 = std::max(0, int(std::floor(g[1] - cfg_.samplesPerCell)));
            int k0 = std::max(0, int(std::floor(g[2] - cfg_.samplesPerCell)));
            int i1 = std::min(b_.dim[0] - 1, int(std::ceil(g[0] + cfg_.samplesPerCell)));
            int j1 = std::min(b_.dim[1] - 1, int(std::ceil(g[1] + cfg_.samplesPerCell)));
            int k1 = std::min(b_.dim[2] - 1, int(std::ceil(g[2] + cfg_.samplesPerCell)));

            for (int k = k0; k <= k1; ++k) for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                auto pw = gridToWorld({ float(i),float(j),float(k) });
                float dx = pw[0] - p.x, dy = pw[1] - p.y, dz = pw[2] - p.z;
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 > r2) continue;
                float w = std::exp(-d2 * inv2sig2);
                int id = idx(i, j, k);
                V_[id][0] += w * p.nx;
                V_[id][1] += w * p.ny;
                V_[id][2] += w * p.nz;
                W[id] += w;
            }
        }
        // Normalize.
        for (size_t t = 0; t < V_.size(); ++t) {
            if (W[t] > 1e-12f) {
                V_[t][0] /= W[t]; V_[t][1] /= W[t]; V_[t][2] /= W[t];
            }
        }
    }

    // Compute right-hand side f = div V using central differences.
    void buildDivergence() {
        const float h = b_.cell;
        const float inv2h = 1.0f / (2.0f * h);
        for (int k = 0; k < b_.dim[2]; ++k) for (int j = 0; j < b_.dim[1]; ++j) for (int i = 0; i < b_.dim[0]; ++i) {
            auto sampleV = [&](int a, int b, int c)->std::array<float, 3> {
                if (inside(a, b, c)) return V_[idx(a, b, c)];
                return V_[idx(std::clamp(a, 0, b_.dim[0] - 1), std::clamp(b, 0, b_.dim[1] - 1), std::clamp(c, 0, b_.dim[2] - 1))];
                };
            auto vxp = sampleV(i + 1, j, k), vxn = sampleV(i - 1, j, k);
            auto vyp = sampleV(i, j + 1, k), vyn = sampleV(i, j - 1, k);
            auto vzp = sampleV(i, j, k + 1), vzn = sampleV(i, j, k - 1);
            float div =
                (vxp[0] - vxn[0]) * inv2h +
                (vyp[1] - vyn[1]) * inv2h +
                (vzp[2] - vzn[2]) * inv2h;
            f_[idx(i, j, k)] = div;
        }
    }

    // Solve the Poisson equation nabla^2 u = f using Successive Over-Relaxation (SOR).
    void solvePoisson() {
        const int nx = b_.dim[0], ny = b_.dim[1], nz = b_.dim[2];
        const float h = b_.cell;
        const float h2 = h * h;
        const float omega = cfg_.omega;
        // Neumann condition approximation: clamp boundary neighbors.
        for (int it = 0; it < cfg_.maxIters; ++it) {
            for (int k = 0; k < nz; ++k) for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
                int id = idx(i, j, k);
                auto U = [&](int a, int b, int c)->float& {
                    return u_[idx(std::clamp(a, 0, nx - 1), std::clamp(b, 0, ny - 1), std::clamp(c, 0, nz - 1))];
                    };
                float sumN = U(i - 1, j, k) + U(i + 1, j, k) + U(i, j - 1, k) + U(i, j + 1, k) + U(i, j, k - 1) + U(i, j, k + 1);
                float u_new = (sumN - h2 * f_[id]) / 6.0f;
                u_[id] = (1.0f - omega) * u_[id] + omega * u_new;
            }
        }
        // Shift DC component (mean) to zero (arbitrary constant from Neumann boundary).
        double mean = std::accumulate(u_.begin(), u_.end(), 0.0) / double(u_.size());
        for (auto& v : u_) v -= float(mean);
    }

    // Extract the iso-surface using a table-less marching tetrahedra approach.
    // Each grid cell is subdivided into 6 tetrahedra.
    void extractIsoSurface(PSTriangleMesh& mesh) const {
        const int nx = b_.dim[0], ny = b_.dim[1], nz = b_.dim[2];
        // Vertex cache omitted (duplicate vertices can be merged as a post-process if needed).
        std::vector<std::array<float, 3>>& verts = mesh.vertices;
        std::vector<std::array<uint32_t, 3>>& faces = mesh.faces;

        auto sampleU = [&](int i, int j, int k)->float {
            return u_[idx(i, j, k)];
            };
        auto idxToWorld = [&](int i, int j, int k)->std::array<float, 3> {
            return gridToWorld({ float(i),float(j),float(k) });
            };
        auto interp = [&](const std::array<float, 3>& p0, float v0,
            const std::array<float, 3>& p1, float v1)->std::array<float, 3> {
                float t = 0.5f;
                float dv = v1 - v0;
                if (std::fabs(dv) > 1e-12f) t = (cfg_.isoLevel - v0) / dv;
                t = std::clamp(t, 0.0f, 1.0f);
                return { p0[0] + t * (p1[0] - p0[0]),
                         p0[1] + t * (p1[1] - p0[1]),
                         p0[2] + t * (p1[2] - p0[2]) };
            };

        // Tetrahedra subdivision pattern (8 corner indices per cell).
        const int tet[6][4] = {
            {0,5,1,6}, {0,1,2,6}, {0,2,3,6},
            {0,3,7,6}, {0,7,4,6}, {0,4,5,6}
        };
        // Local coordinates of each cell corner.
        const int vLUT[8][3] = {
            {0,0,0},{1,0,0},{1,1,0},{0,1,0},
            {0,0,1},{1,0,1},{1,1,1},{0,1,1}
        };

        for (int k = 0; k < nz - 1; ++k) for (int j = 0; j < ny - 1; ++j) for (int i = 0; i < nx - 1; ++i) {
            std::array<std::array<float, 3>, 8> P;
            std::array<float, 8> V;
            for (int c = 0; c < 8; ++c) {
                int ii = i + vLUT[c][0];
                int jj = j + vLUT[c][1];
                int kk = k + vLUT[c][2];
                P[c] = idxToWorld(ii, jj, kk);
                V[c] = sampleU(ii, jj, kk) - cfg_.isoLevel;
            }
            for (int t = 0; t < 6; ++t) {
                int a = tet[t][0], b = tet[t][1], c = tet[t][2], d = tet[t][3];
                float va = V[a], vb = V[b], vc = V[c], vd = V[d];
                int mask = (va > 0) | ((vb > 0) << 1) | ((vc > 0) << 2) | ((vd > 0) << 3);
                if (mask == 0 || mask == 15) continue; // No intersection.

                auto emitTri = [&](const std::array<float, 3>& A,
                    const std::array<float, 3>& B,
                    const std::array<float, 3>& C) {
                        uint32_t base = (uint32_t)verts.size();
                        verts.push_back(A); verts.push_back(B); verts.push_back(C);
                        faces.push_back({ base, base + 1, base + 2 });
                    };

                auto e = [&](int i0, int i1)->std::array<float, 3> {
                    return interp(P[i0], V[i0], P[i1], V[i1]);
                    };

                // Vertex indices: 0->a, 1->b, 2->c, 3->d; edges (0-1),(0-2),(0-3),(1-2),(1-3),(2-3).
                auto A = [&](int x) { return (x == 0) ? a : (x == 1) ? b : (x == 2) ? c : d; };

                switch (mask) {
                case 1: case 14: {
                    bool inv = (mask == 14);
                    auto p0 = e(A(0), A(1));
                    auto p1 = e(A(0), A(2));
                    auto p2 = e(A(0), A(3));
                    if (!inv) emitTri(p0, p1, p2); else emitTri(p0, p2, p1);
                } break;
                case 2: case 13: {
                    bool inv = (mask == 13);
                    auto p0 = e(A(1), A(0));
                    auto p1 = e(A(1), A(2));
                    auto p2 = e(A(1), A(3));
                    if (!inv) emitTri(p0, p1, p2); else emitTri(p0, p2, p1);
                } break;
                case 4: case 11: {
                    bool inv = (mask == 11);
                    auto p0 = e(A(2), A(0));
                    auto p1 = e(A(2), A(1));
                    auto p2 = e(A(2), A(3));
                    if (!inv) emitTri(p0, p1, p2); else emitTri(p0, p2, p1);
                } break;
                case 8: case 7: {
                    bool inv = (mask == 7);
                    auto p0 = e(A(3), A(0));
                    auto p1 = e(A(3), A(1));
                    auto p2 = e(A(3), A(2));
                    if (!inv) emitTri(p0, p1, p2); else emitTri(p0, p2, p1);
                } break;
                case 3: case 12: {
                    bool inv = (mask == 12);
                    auto p0 = e(A(0), A(2));
                    auto p1 = e(A(0), A(3));
                    auto p2 = e(A(1), A(2));
                    auto p3 = e(A(1), A(3));
                    if (!inv) { emitTri(p0, p1, p3); emitTri(p0, p3, p2); }
                    else { emitTri(p0, p2, p3); emitTri(p0, p3, p1); }
                } break;
                case 5: case 10: {
                    bool inv = (mask == 10);
                    auto p0 = e(A(0), A(1));
                    auto p1 = e(A(0), A(3));
                    auto p2 = e(A(2), A(1));
                    auto p3 = e(A(2), A(3));
                    if (!inv) { emitTri(p0, p1, p3); emitTri(p0, p3, p2); }
                    else { emitTri(p0, p2, p3); emitTri(p0, p3, p1); }
                } break;
                case 6: case 9: {
                    bool inv = (mask == 9);
                    auto p0 = e(A(0), A(1));
                    auto p1 = e(A(0), A(2));
                    auto p2 = e(A(3), A(1));
                    auto p3 = e(A(3), A(2));
                    if (!inv) { emitTri(p0, p1, p3); emitTri(p0, p3, p2); }
                    else { emitTri(p0, p2, p3); emitTri(p0, p3, p1); }
                } break;
                default: break;
                }
            }
        }
    }

    std::array<float, 3> worldToGrid(const std::array<float, 3>& p) const {
        return { (p[0] - b_.min[0]) / b_.cell,
                 (p[1] - b_.min[1]) / b_.cell,
                 (p[2] - b_.min[2]) / b_.cell };
    }
    std::array<float, 3> gridToWorld(const std::array<float, 3>& g) const {
        return { b_.min[0] + g[0] * b_.cell,
                 b_.min[1] + g[1] * b_.cell,
                 b_.min[2] + g[2] * b_.cell };
    }
};

// Defined out-of-line: Config{} needs PoissonSurface (the enclosing class) to be
// complete before its nested Config's default member initializers can be used
// (MSVC is lenient about this; clang/GCC require strict standard conformance).
inline PoissonSurface::PoissonSurface() : PoissonSurface(Config{}) {}
