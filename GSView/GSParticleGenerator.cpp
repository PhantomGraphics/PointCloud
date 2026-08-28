#include "GSParticleGenerator.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/common.hpp>

#include <algorithm>
#include <cmath>

namespace {

float sigmoid(float x)
{
	return 1.0f / (1.0f + std::exp(-x));
}

} // namespace

namespace GSView {

Phantom::Volume::ParticleSet GSParticleGenerator::generate(const Phantom::PointCloud::GSPointCloud& gs) const
{
	Phantom::Volume::ParticleSet set;
	if (gs.points.empty()) {
		return set;
	}

	size_t totalEstimate = 0;
	for (const auto& p : gs.points) {
		const float opacity = sigmoid(p.opacity);
		const float scaledCount = densityScale_ * opacity * static_cast<float>(std::max(1, maxParticlesPerSplat_));
		totalEstimate += static_cast<size_t>(std::max(1, static_cast<int>(std::lround(scaledCount))));
	}
	set.particles.reserve(totalEstimate);

	std::normal_distribution<float> normal(0.0f, 1.0f);

	for (const auto& p : gs.points) {
		const float opacity = sigmoid(p.opacity);
		const float scaledCount = densityScale_ * opacity * static_cast<float>(std::max(1, maxParticlesPerSplat_));
		const int particleCount = std::max(1, static_cast<int>(std::lround(scaledCount)));

		glm::mat3 L = cholesky(computeCovariance(p));

		glm::vec3 baseColor(
			glm::clamp(0.5f + 0.28209f * p.f_dc[0], 0.0f, 1.0f),
			glm::clamp(0.5f + 0.28209f * p.f_dc[1], 0.0f, 1.0f),
			glm::clamp(0.5f + 0.28209f * p.f_dc[2], 0.0f, 1.0f));

		const float alphaPerParticle = opacity / static_cast<float>(particleCount);
		const glm::vec3 weightedColor = baseColor * alphaPerParticle;

		const glm::vec3 center(p.x, p.y, p.z);
		for (int i = 0; i < particleCount; ++i) {
			const glm::vec3 n(normal(rng_), normal(rng_), normal(rng_));
			Phantom::Volume::Particle particle;
			particle.pos = center + L * n;
			particle.color = weightedColor;
			set.particles.push_back(particle);
		}
	}

	return set;
}

glm::mat3 GSParticleGenerator::computeCovariance(const Phantom::PointCloud::GSPoint& p) const
{
	const glm::quat q(p.rot[0], p.rot[1], p.rot[2], p.rot[3]);
	const glm::mat3 R = glm::mat3_cast(glm::normalize(q));

	const float sx = std::exp(p.scale[0]);
	const float sy = std::exp(p.scale[1]);
	const float sz = std::exp(p.scale[2]);

	glm::mat3 S2(1.0f);
	S2[0][0] = sx * sx;
	S2[1][1] = sy * sy;
	S2[2][2] = sz * sz;

	glm::mat3 cov = R * S2 * glm::transpose(R);

	const float epsilon = 1e-6f;
	cov[0][0] = std::max(cov[0][0], epsilon);
	cov[1][1] = std::max(cov[1][1], epsilon);
	cov[2][2] = std::max(cov[2][2], epsilon);
	return cov;
}

glm::mat3 GSParticleGenerator::cholesky(const glm::mat3& A) const
{
	const float eps = 1e-8f;

	const float a00 = std::max(A[0][0], eps);
	const float l00 = std::sqrt(a00);

	const float l10 = A[0][1] / l00;
	const float l20 = A[0][2] / l00;

	const float a11 = std::max(A[1][1] - l10 * l10, eps);
	const float l11 = std::sqrt(a11);

	const float l21 = (A[1][2] - l20 * l10) / l11;

	const float a22 = std::max(A[2][2] - l20 * l20 - l21 * l21, eps);
	const float l22 = std::sqrt(a22);

	glm::mat3 L(0.0f);
	L[0][0] = l00;
	L[0][1] = l10;
	L[0][2] = l20;
	L[1][1] = l11;
	L[1][2] = l21;
	L[2][2] = l22;
	return L;
}

} // namespace GSView
