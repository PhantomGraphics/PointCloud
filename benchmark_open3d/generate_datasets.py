"""Generate synthetic point cloud datasets (binary little-endian PLY, float32 x/y/z)
for benchmarking this repo's PointCloud library against Open3D.

No open3d dependency here - pure numpy, so it can run under either python install.
"""
import numpy as np
import os

OUT_DIR = os.path.join(os.path.dirname(__file__), "datasets")
os.makedirs(OUT_DIR, exist_ok=True)

SIZES = [50_000, 200_000, 1_000_000]
SEED = 42


def write_ply_binary(path, points: np.ndarray):
    assert points.dtype == np.float32
    assert points.shape[1] == 3
    n = points.shape[0]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(points).tobytes())


def gen_cube_uniform(rng, n):
    return rng.uniform(-5.0, 5.0, size=(n, 3)).astype(np.float32)


def gen_clustered_blobs(rng, n, n_blobs=8, target_density=40.0):
    # Uniform-in-sphere blobs (not Gaussian) sized so the average point density
    # stays roughly constant across dataset sizes - a Gaussian blob's density
    # spikes sharply near its center, which blows up eps-radius neighbor counts
    # (and DBSCAN/region-growing runtime) at large N regardless of implementation.
    centers = rng.uniform(-8.0, 8.0, size=(n_blobs, 3))
    counts = np.full(n_blobs, n // n_blobs)
    counts[-1] += n - counts.sum()
    parts = []
    for c, cnt in zip(centers, counts):
        radius = (cnt / ((4.0 / 3.0) * np.pi * target_density)) ** (1.0 / 3.0)
        # sample uniformly inside a sphere of `radius`
        direction = rng.normal(size=(cnt, 3))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        r = radius * rng.uniform(size=cnt) ** (1.0 / 3.0)
        parts.append(c + direction * r[:, None])
    pts = np.concatenate(parts, axis=0).astype(np.float32)
    rng.shuffle(pts)
    return pts


def gen_plane_with_outliers(rng, n, outlier_ratio=0.15):
    n_outliers = int(n * outlier_ratio)
    n_plane = n - n_outliers
    xy = rng.uniform(-10.0, 10.0, size=(n_plane, 2))
    noise = rng.normal(0.0, 0.02, size=n_plane)
    # plane: z = 0.3*x - 0.2*y + 1.0 (+ noise)
    z = 0.3 * xy[:, 0] - 0.2 * xy[:, 1] + 1.0 + noise
    plane_pts = np.column_stack([xy[:, 0], xy[:, 1], z])
    outliers = rng.uniform(-10.0, 10.0, size=(n_outliers, 3))
    pts = np.concatenate([plane_pts, outliers], axis=0).astype(np.float32)
    rng.shuffle(pts)
    return pts


def gen_sphere_surface(rng, n, radius=5.0, noise=0.02):
    # uniform on sphere via normalized gaussian
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    pts = v * radius
    pts += rng.normal(0.0, noise, size=(n, 3))
    return pts.astype(np.float32)


def rotation_matrix(rx, ry, rz):
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def main():
    rng = np.random.default_rng(SEED)
    manifest = []
    for n in SIZES:
        cube = gen_cube_uniform(rng, n)
        p = os.path.join(OUT_DIR, f"cube_uniform_{n}.ply")
        write_ply_binary(p, cube)
        manifest.append((p, n))

        blobs = gen_clustered_blobs(rng, n)
        p = os.path.join(OUT_DIR, f"clustered_blobs_{n}.ply")
        write_ply_binary(p, blobs)
        manifest.append((p, n))

        plane = gen_plane_with_outliers(rng, n)
        p = os.path.join(OUT_DIR, f"plane_outliers_{n}.ply")
        write_ply_binary(p, plane)
        manifest.append((p, n))

        target = gen_sphere_surface(rng, n)
        p = os.path.join(OUT_DIR, f"icp_target_{n}.ply")
        write_ply_binary(p, target)
        manifest.append((p, n))

        R = rotation_matrix(0.15, 0.25, -0.10)
        t = np.array([0.4, -0.3, 0.6], dtype=np.float32)
        source = (target @ R.T).astype(np.float32) + t
        source += rng.normal(0.0, 0.01, size=source.shape).astype(np.float32)
        p = os.path.join(OUT_DIR, f"icp_source_{n}.ply")
        write_ply_binary(p, source)
        manifest.append((p, n))

    print(f"Generated {len(manifest)} PLY files in {OUT_DIR}")
    for p, n in manifest:
        print(f"  {os.path.basename(p):40s} n={n}")


if __name__ == "__main__":
    main()
