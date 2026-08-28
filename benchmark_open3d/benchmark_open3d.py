"""Benchmark Open3D on the same synthetic datasets/parameters used for the
Crystal2024 PointCloud module C++ benchmark (benchmark_pointcloud.cpp), for a
speed comparison. See README.md for how to run this.

Run with: py -3.11 benchmark_open3d.py   (open3d has no Python 3.13 wheel yet)
"""
import time
import os
import csv
import numpy as np
import open3d as o3d

DATA_DIR = os.path.join(os.path.dirname(__file__), "datasets")
SIZES = [50_000, 200_000, 1_000_000]
RESULTS_CSV = os.path.join(os.path.dirname(__file__), "open3d_results.csv")

VOXEL_SIZE = 0.3
NORMAL_RADIUS = 0.5
SOR_K = 20
SOR_STD_RATIO = 2.0
RANSAC_ITERS = 200
RANSAC_DIST_THRESHOLD = 0.02
ICP_MAX_ITER = 30
ICP_MAX_CORR_DIST = 3.0
DBSCAN_EPS = 0.5
DBSCAN_MIN_POINTS = 10


def load_pcd(path):
    return o3d.io.read_point_cloud(path)


def timeit(fn, repeats=1):
    best = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        dt = time.perf_counter() - t0
        best = dt if best is None else min(best, dt)
    return best


def bench_downsample(n, results):
    pcd = load_pcd(os.path.join(DATA_DIR, f"cube_uniform_{n}.ply"))
    out = {}
    dt = timeit(lambda: out.setdefault("r", pcd.voxel_down_sample(VOXEL_SIZE)))
    results.append(("voxel_downsample", n, dt, len(out["r"].points)))
    print(f"[downsample]   n={n:>8}  {dt:8.4f}s  out={len(out['r'].points)}")


def bench_normals(n, results):
    pcd = load_pcd(os.path.join(DATA_DIR, f"cube_uniform_{n}.ply"))

    def run():
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(NORMAL_RADIUS))

    dt = timeit(run)
    results.append(("normal_estimation", n, dt, n))
    print(f"[normals]      n={n:>8}  {dt:8.4f}s")


def bench_sor(n, results):
    pcd = load_pcd(os.path.join(DATA_DIR, f"cube_uniform_{n}.ply"))
    out = {}

    def run():
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=SOR_K, std_ratio=SOR_STD_RATIO)
        out["ind"] = ind

    dt = timeit(run)
    results.append(("sor_filter", n, dt, len(out["ind"])))
    print(f"[sor]          n={n:>8}  {dt:8.4f}s  inliers={len(out['ind'])}")


def bench_ransac_plane(n, results):
    pcd = load_pcd(os.path.join(DATA_DIR, f"plane_outliers_{n}.ply"))
    out = {}

    def run():
        model, inliers = pcd.segment_plane(
            distance_threshold=RANSAC_DIST_THRESHOLD,
            ransac_n=3,
            num_iterations=RANSAC_ITERS,
        )
        out["inliers"] = inliers

    dt = timeit(run)
    results.append(("ransac_plane", n, dt, len(out["inliers"])))
    print(f"[ransac_plane] n={n:>8}  {dt:8.4f}s  inliers={len(out['inliers'])}")


def bench_dbscan(n, results):
    pcd = load_pcd(os.path.join(DATA_DIR, f"clustered_blobs_{n}.ply"))
    out = {}

    def run():
        labels = np.array(pcd.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS))
        out["labels"] = labels

    dt = timeit(run)
    n_clusters = len(set(out["labels"])) - (1 if -1 in out["labels"] else 0)
    results.append(("dbscan", n, dt, n_clusters))
    print(f"[dbscan]       n={n:>8}  {dt:8.4f}s  clusters={n_clusters}")


def bench_icp(n, results):
    source = load_pcd(os.path.join(DATA_DIR, f"icp_source_{n}.ply"))
    target = load_pcd(os.path.join(DATA_DIR, f"icp_target_{n}.ply"))
    out = {}

    def run():
        reg = o3d.pipelines.registration.registration_icp(
            source, target, ICP_MAX_CORR_DIST, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-12, relative_rmse=1e-12, max_iteration=ICP_MAX_ITER
            ),
        )
        out["reg"] = reg

    dt = timeit(run)
    results.append(("icp_point_to_point", n, dt, out["reg"].fitness))
    print(f"[icp]          n={n:>8}  {dt:8.4f}s  fitness={out['reg'].fitness:.6f}")


def main():
    print(f"open3d version: {o3d.__version__}")
    results = []
    for n in SIZES:
        bench_downsample(n, results)
        bench_normals(n, results)
        bench_sor(n, results)
        bench_ransac_plane(n, results)
        bench_dbscan(n, results)
        bench_icp(n, results)

    with open(RESULTS_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["algorithm", "n_points", "time_sec", "metric"])
        for row in results:
            w.writerow(row)
    print(f"\nWrote {RESULTS_CSV}")


if __name__ == "__main__":
    main()
