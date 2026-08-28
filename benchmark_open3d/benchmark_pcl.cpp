// Standalone speed benchmark for PCL (Point Cloud Library) 1.15.1, using the same
// synthetic PLY datasets and parameters as benchmark_pointcloud.cpp / benchmark_open3d.py
// (see README.md in this folder). Not part of the repo's own PointCloud module - this
// links only against the official PCL AllInOne prebuilt binaries, plus a tiny self-contained
// PLY reader (avoids mixing PCL's VS2022(vc143) prebuilt libs with this repo's VS2026(v145)
// libs at link time).

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>

static const float VOXEL_SIZE = 0.3f;
static const double NORMAL_RADIUS = 0.5;
static const int SOR_K = 20;
static const double SOR_STD_RATIO = 2.0;
static const int RANSAC_ITERS = 200;
static const double RANSAC_DIST_THRESHOLD = 0.02;
static const int ICP_MAX_ITER = 30;
static const double ICP_MAX_CORR_DIST = 3.0;

// Minimal reader for the fixed binary_little_endian PLY format produced by
// generate_datasets.py: header lines, then N * (float32 x, float32 y, float32 z).
static pcl::PointCloud<pcl::PointXYZ>::Ptr loadPly(const std::string& path)
{
	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	std::ifstream in(path, std::ios::binary);
	if (!in) {
		std::fprintf(stderr, "FAILED to open %s\n", path.c_str());
		return cloud;
	}
	std::string line;
	size_t n = 0;
	while (std::getline(in, line)) {
		if (line.rfind("element vertex", 0) == 0) {
			n = static_cast<size_t>(std::stoull(line.substr(15)));
		}
		if (line == "end_header" || line == "end_header\r") {
			break;
		}
	}
	cloud->resize(n);
	std::vector<float> buf(n * 3);
	in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size() * sizeof(float)));
	for (size_t i = 0; i < n; ++i) {
		(*cloud)[i].x = buf[i * 3 + 0];
		(*cloud)[i].y = buf[i * 3 + 1];
		(*cloud)[i].z = buf[i * 3 + 2];
	}
	return cloud;
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
	auto cloud = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
	double dt = timeit([&]() {
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud);
		vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
		vg.filter(*out);
	});
	std::printf("[downsample]   n=%8zu  %8.4fs  out=%zu\n", n, dt, out->size()); std::fflush(stdout);
	g_results.push_back({ "voxel_downsample", n, dt, static_cast<double>(out->size()) });
}

static void benchNormals(const std::string& dataDir, size_t n)
{
	flushLog("normals", n);
	auto cloud = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	double dt = timeit([&]() {
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(NORMAL_RADIUS);
		ne.compute(*normals);
	});
	std::printf("[normals]      n=%8zu  %8.4fs\n", n, dt); std::fflush(stdout);
	g_results.push_back({ "normal_estimation", n, dt, static_cast<double>(normals->size()) });
}

static void benchSor(const std::string& dataDir, size_t n)
{
	flushLog("sor", n);
	auto cloud = loadPly(dataDir + "\\cube_uniform_" + std::to_string(n) + ".ply");
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
	double dt = timeit([&]() {
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(SOR_K);
		sor.setStddevMulThresh(SOR_STD_RATIO);
		sor.filter(*out);
	});
	std::printf("[sor]          n=%8zu  %8.4fs  inliers=%zu\n", n, dt, out->size()); std::fflush(stdout);
	g_results.push_back({ "sor_filter", n, dt, static_cast<double>(out->size()) });
}

static void benchRansacPlane(const std::string& dataDir, size_t n)
{
	flushLog("ransac_plane", n);
	auto cloud = loadPly(dataDir + "\\plane_outliers_" + std::to_string(n) + ".ply");
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	double dt = timeit([&]() {
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(RANSAC_ITERS);
		seg.setDistanceThreshold(RANSAC_DIST_THRESHOLD);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
	});
	std::printf("[ransac_plane] n=%8zu  %8.4fs  inliers=%zu\n", n, dt, inliers->indices.size()); std::fflush(stdout);
	g_results.push_back({ "ransac_plane", n, dt, static_cast<double>(inliers->indices.size()) });
}

static void benchIcp(const std::string& dataDir, size_t n)
{
	flushLog("icp", n);
	auto source = loadPly(dataDir + "\\icp_source_" + std::to_string(n) + ".ply");
	auto target = loadPly(dataDir + "\\icp_target_" + std::to_string(n) + ".ply");
	pcl::PointCloud<pcl::PointXYZ> final;
	double fitness = 0.0;
	double dt = timeit([&]() {
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(source);
		icp.setInputTarget(target);
		icp.setMaximumIterations(ICP_MAX_ITER);
		icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
		icp.setTransformationEpsilon(1e-12);
		icp.setEuclideanFitnessEpsilon(-1.0);
		icp.align(final);
		fitness = icp.getFitnessScore();
	});
	std::printf("[icp]          n=%8zu  %8.4fs  fitness=%.6f\n", n, dt, fitness); std::fflush(stdout);
	g_results.push_back({ "icp_point_to_point", n, dt, fitness });
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
		benchIcp(dataDir, n);
	}

	std::ofstream out(dataDir + "\\..\\pcl_results.csv");
	out << "algorithm,n_points,time_sec,metric\n";
	for (auto& r : g_results) {
		out << r.algo << "," << r.n << "," << r.sec << "," << r.metric << "\n";
	}
	std::printf("\nWrote pcl_results.csv\n");
	return 0;
}
