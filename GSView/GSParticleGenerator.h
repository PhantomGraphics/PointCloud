#pragma once

#include "../PointCloud/GSPointCloud.h"
#include "../../CGLib/Volume/VolumeRenderer/ParticleSet.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <random>

namespace GSView {

class GSParticleGenerator {
public:
	void setDensityScale(float s) { densityScale_ = s; }
	void setMaxParticlesPerSplat(int n) { maxParticlesPerSplat_ = n; }

	Phantom::Volume::ParticleSet generate(const Phantom::PointCloud::GSPointCloud& gs) const;

private:
	glm::mat3 computeCovariance(const Phantom::PointCloud::GSPoint& p) const;
	glm::mat3 cholesky(const glm::mat3& A) const;

	float densityScale_ = 1.0f;
	int maxParticlesPerSplat_ = 8;
	mutable std::mt19937 rng_{ 42u };
};

} // namespace GSView
