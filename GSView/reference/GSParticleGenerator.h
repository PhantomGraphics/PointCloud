#pragma once

// -----------------------------------------------------------------------------
// CPU reference implementation of the "3D Gaussian -> world-space particles"
// (PBVR3DExperimental) generation. This is NOT on the GSView render path — the
// production path is the GPU compute shader in GSComputePBVR / gs_pbvr_gen.comp.
//
// It is kept as a deterministic, dependency-light reference so PointCloudTest can
// pin the particle-count formula and the covariance / Cholesky sampling maths, and
// so Phase 1 of docs/todo/PLAN_gsview_gaussian_point_pbvr.md has a starting point
// for the CPU oracle. Self-contained: depends only on glm and GSPointCloud.
// -----------------------------------------------------------------------------

#include "../../PointCloud/GSPointCloud.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <cstdint>
#include <random>
#include <vector>

namespace GSView::reference {

struct CpuParticle {
	glm::vec3 pos{};
	glm::vec3 color{};   // baseColor * (opacity / particleCount), straight (not premultiplied by gaussian)
};

struct CpuParticleSet {
	std::vector<CpuParticle> particles;
	std::size_t count() const { return particles.size(); }
};

class GSParticleGenerator {
public:
	void setDensityScale(float s) { densityScale_ = s; }
	void setMaxParticlesPerSplat(int n) { maxParticlesPerSplat_ = n; }
	void setSeed(std::uint32_t seed) { seed_ = seed; }

	// Deterministic: reseeds its RNG at the start of every call, so repeated calls
	// with the same inputs produce byte-identical output.
	CpuParticleSet generate(const Phantom::PointCloud::GSPointCloud& gs) const;

	// Per-splat particle count, matching GSComputePBVR::particleCountForSplat() and
	// gs_pbvr_gen.comp: clamp(round(densityScale * sigmoid(opacity) * maxPPS), 0, maxPPS).
	int particleCountForSplat(const Phantom::PointCloud::GSPoint& p) const;

private:
	glm::mat3 computeCovariance(const Phantom::PointCloud::GSPoint& p) const;
	glm::mat3 cholesky(const glm::mat3& A) const;

	float densityScale_ = 1.0f;
	int maxParticlesPerSplat_ = 8;
	std::uint32_t seed_ = 42u;
	mutable std::mt19937 rng_{ 42u };
};

} // namespace GSView::reference
