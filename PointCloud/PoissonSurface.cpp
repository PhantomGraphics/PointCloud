#include "PoissonSurface.h"
#include <iostream>

/*
int main() {
    // 例: 球面サンプル点群（法線は外向き）
    std::vector<PSPoint> pts;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        float u = float(rand()) / RAND_MAX;
        float v = float(rand()) / RAND_MAX;
        float th = 2.0f * 3.1415926f * u;
        float ph = std::acos(2.0f * v - 1.0f);
        float x = std::sin(ph) * std::cos(th);
        float y = std::sin(ph) * std::sin(th);
        float z = std::cos(ph);
        pts.push_back({ x,y,z, x,y,z });
    }

    PoissonSurface::Config cfg;
    cfg.resolution = 96;
    cfg.maxIters = 150;
    cfg.samplesPerCell = 1.2f;

    PoissonSurface ps(cfg);
    PSTriangleMesh mesh = ps.reconstruct(pts);

    std::cout << "verts=" << mesh.vertices.size()
        << " faces=" << mesh.faces.size() << "\n";

    // 簡易PLY出力
    FILE* f = fopen("poisson_mesh.ply", "wb");
    fprintf(f, "ply\nformat ascii 1.0\n");
    fprintf(f, "element vertex %zu\nproperty float x\nproperty float y\nproperty float z\n", mesh.vertices.size());
    fprintf(f, "element face %zu\nproperty list uchar uint vertex_indices\nend_header\n", mesh.faces.size());
    for (auto& p : mesh.vertices) fprintf(f, "%f %f %f\n", p[0], p[1], p[2]);
    for (auto& t : mesh.faces) fprintf(f, "3 %u %u %u\n", t[0], t[1], t[2]);
    fclose(f);
}
*/