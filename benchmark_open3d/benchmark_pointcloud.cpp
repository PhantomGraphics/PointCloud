// Standalone speed benchmark for this repo's PointCloud module (not part of the
// normal solution build - see README.md/build_benchmark.ps1 in this folder).
// Reads the same synthetic PLY datasets as benchmark_open3d.py and times the
// equivalent PointCloud calls with matching parameters, for a speed comparison.

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "PointCloud/PointCloud/PLYFileReader.h"
#include "PointCloud/PointCloud/DownSampler.h"
#include "PointCloud/PointCloud/NormalEstimator.h"
#include "PointCloud/PointCloud/SORFilter.h"
#include "PointCloud/PointCloud/RansacPlaneDetector.h"
#include "PointCloud/PointCloud/DBSCAN.h"
#include "PointCloud/PointCloud/ICPRegistration.h"

using namespace Phantom::PC;
using namespace Phantom::Math;

static const double VOXEL_SIZE = 0.3;
static const double NORMAL_RADIUS = 0.5;
static const int SOR_K = 20;
static const float SOR_STD_RATIO = 2.0f;
static const int RANSAC_ITERS = 200;
static const float RANSAC_DIST_THRESHOLD = 0.02f;
static const int ICP_MAX_ITER = 30;
static const float ICP_MAX_CORR_DIST = 3.0f;
static const double DBSCAN_EPS = 0.5;
static const int DBSCAN_MIN_PTS = 10;

static std::vector<Vector3df> loadPly(const std::string& path)
{
	PLYFileReader reader;
	if (!reader.read(path)) {
		std::fprintf(stderr, "FAILED to read %s: %s\n", path.c_str(), reader.getLastError().c_str());
		return {};
	}
	return reader.getFile().getPoints();
}

template<typename F>
static double timeit(F&& f)
{
	auto t0 = std::chrono::high_resolution_clock::now();
	f();
	auto t1 = std::chrono::high_resolution_clock::now();
	return std::chrono::duration<double>(t1 - t0).count();
}

static void flushLog(const char* stage, size_t n)
{
	std::printf("  ... starting %s n=%zu\n", stage, n);
	std::fflush(stdout);
}

struct Row { std::string algo; size_t n; double sec; double metric; };
static std::vector<Row> g_results;

static void benchDownsample(const std::string& dataDir, size_t n)
{
	flushLog("downsample", n);
	auto pts = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	DownSampler sampler;
	for (auto& p : pts) sampler.add(p);
	size_t outCount = 0;
	double dt = timeit([&]() {
		sampler.execute(VOXEL_SIZE);
		outCount = sampler.getDownSampled().size();
	});
	std::printf("[downsample]   n=%8zu  %8.4fs  out=%zu\n", n, dt, outCount); std::fflush(stdout);
	g_results.push_back({ "voxel_downsample", n, dt, (double)outCount });
}

static void benchNormals(const std::string& dataDir, size_t n)
{
	flushLog("normals", n);
	auto pts = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	NormalEstimator est;
	for (auto& p : pts) est.add(p);
	double dt = timeit([&]() {
		est.estimate(NORMAL_RADIUS);
	});
	std::printf("[normals]      n=%8zu  %8.4fs\n", n, dt); std::fflush(stdout);
	g_results.push_back({ "normal_estimation", n, dt, (double)n });
}

static void benchSor(const std::string& dataDir, size_t n)
{
	// SORFilter::execute() now uses Phantom::Space::KDTree for k-NN search (O(n log n)) instead
	// of the old brute-force O(n^2) all-pairs scan, so unlike before there's no need to skip 1M.
	flushLog("sor", n);
	auto pts = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	SORFilter filter;
	for (auto& p : pts) filter.add(p);
	size_t inlierCount = 0;
	double dt = timeit([&]() {
		filter.execute(SOR_K, SOR_STD_RATIO);
		inlierCount = filter.getInlierIndices().size();
	});
	std::printf("[sor]          n=%8zu  %8.4fs  inliers=%zu\n", n, dt, inlierCount); std::fflush(stdout);
	g_results.push_back({ "sor_filter", n, dt, (double)inlierCount });
}

static void benchRansacPlane(const std::string& dataDir, size_t n)
{
	flushLog("ransac_plane", n);
	auto pts = loadPly(dataDir + "\\plane_outliers_" + std::to_string(n) + ".ply");
	RansacPlaneDetector detector(42u);
	RansacPlaneDetector::PlaneModel model;
	bool ok = false;
	double dt = timeit([&]() {
		ok = detector.detect(pts, model, RANSAC_ITERS, RANSAC_DIST_THRESHOLD, 50);
	});
	std::printf("[ransac_plane] n=%8zu  %8.4fs  inliers=%zu ok=%d\n", n, dt, model.inliers.size(), ok ? 1 : 0); std::fflush(stdout);
	g_results.push_back({ "ransac_plane", n, dt, (double)model.inliers.size() });
}

static void benchDbscan(const std::string& dataDir, size_t n)
{
	flushLog("dbscan", n);
	auto pts = loadPly(dataDir + "\\clustered_blobs_" + std::to_string(n) + ".ply");
	std::vector<Point> dbscanPts;
	dbscanPts.reserve(pts.size());
	for (auto& p : pts) dbscanPts.emplace_back(p[0], p[1], p[2]);

	DBSCANClustering dbscan;
	double dt = timeit([&]() {
		dbscan.cluster(dbscanPts, DBSCAN_EPS, DBSCAN_MIN_PTS);
	});
	int maxCluster = 0;
	for (auto& p : dbscanPts) if (p.clusterID > maxCluster) maxCluster = p.clusterID;
	std::printf("[dbscan]       n=%8zu  %8.4fs  clusters=%d\n", n, dt, maxCluster); std::fflush(stdout);
	g_results.push_back({ "dbscan", n, dt, (double)maxCluster });
}

static void benchIcp(const std::string& dataDir, size_t n)
{
	flushLog("icp", n);
	auto source = loadPly(dataDir + "\\icp_source_" + std::to_string(n) + ".ply");
	auto target = loadPly(dataDir + "\\icp_target_" + std::to_string(n) + ".ply");

	ICPRegistration icp;
	ICPRegistration::Result result;
	double dt = timeit([&]() {
		icp.align(source, target, result, ICP_MAX_ITER, 1e-12f, ICP_MAX_CORR_DIST,
			ICPRegistration::RobustKernel::None, 1.0f, false);
	});
	std::printf("[icp]          n=%8zu  %8.4fs  fitness=%.6f iters=%d\n", n, dt, result.fitness, result.iterations); std::fflush(stdout);
	g_results.push_back({ "icp_point_to_point", n, dt, (double)result.fitness });
}

int main(int argc, char** argv)
{
	std::string dataDir = argc > 1 ? argv[1] : ".";
	std::vector<size_t> sizes = { 50000, 200000, 1000000 };

	for (auto n : sizes) {
		benchDownsample(dataDir, n);
		benchNormals(dataDir, n);
		benchSor(dataDir, n);
		benchRansacPlane(dataDir, n);
		benchDbscan(dataDir, n);
		benchIcp(dataDir, n);
	}

	std::ofstream out(dataDir + "\\..\\cpp_results.csv");
	out << "algorithm,n_points,time_sec,metric\n";
	for (auto& r : g_results) {
		out << r.algo << "," << r.n << "," << r.sec << "," << r.metric << "\n";
	}
	std::printf("\nWrote cpp_results.csv\n");
	return 0;
}
