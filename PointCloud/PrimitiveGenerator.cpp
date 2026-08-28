#include "PrimitiveGenerator.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace Phantom {
	namespace PC {

		namespace {

			constexpr float kPi = 3.14159265358979323846f;

			std::mt19937 makeRng(const uint32_t seed)
			{
				if (seed != 0) {
					return std::mt19937(seed);
				}
				std::random_device rd;
				return std::mt19937(rd());
			}

		}

		std::vector<Math::Vector3df> PrimitiveGenerator::generateSphere(
			const Math::Vector3df& center, const float radius,
			const int pointCount, const float noise, const uint32_t seed)
		{
			std::vector<Math::Vector3df> result;
			if (radius <= 0.0f || pointCount <= 0) {
				return result;
			}
			result.resize(static_cast<size_t>(pointCount));

			std::mt19937 rng = makeRng(seed);
			std::normal_distribution<float> noiseDist(0.0f, noise > 0.0f ? noise : 1.0f);
			std::uniform_real_distribution<float> cosDist(-1.0f, 1.0f);
			std::uniform_real_distribution<float> phiDist(0.0f, 2.0f * kPi);

			for (int i = 0; i < pointCount; ++i) {
				const float z = cosDist(rng);
				const float sinTheta = std::sqrt(std::max(0.0f, 1.0f - z * z));
				const float phi = phiDist(rng);
				const float nx = sinTheta * std::cos(phi);
				const float ny = sinTheta * std::sin(phi);
				const float r = radius + (noise > 0.0f ? noiseDist(rng) : 0.0f);
				result[i] = Math::Vector3df(center.x + r * nx, center.y + r * ny, center.z + r * z);
			}
			return result;
		}

		std::vector<Math::Vector3df> PrimitiveGenerator::generateCylinder(
			const Math::Vector3df& center, const float radius, const float height,
			const int pointCount, const float noise, const uint32_t seed)
		{
			std::vector<Math::Vector3df> result;
			if (radius <= 0.0f || height <= 0.0f || pointCount <= 0) {
				return result;
			}
			result.resize(static_cast<size_t>(pointCount));

			std::mt19937 rng = makeRng(seed);
			std::normal_distribution<float> noiseDist(0.0f, noise > 0.0f ? noise : 1.0f);
			std::uniform_real_distribution<float> thetaDist(0.0f, 2.0f * kPi);
			std::uniform_real_distribution<float> zDist(0.0f, height);

			for (int i = 0; i < pointCount; ++i) {
				const float theta = thetaDist(rng);
				const float zv = zDist(rng);
				const float r = radius + (noise > 0.0f ? noiseDist(rng) : 0.0f);
				result[i] = Math::Vector3df(
					center.x + r * std::cos(theta),
					center.y + r * std::sin(theta),
					center.z + zv);
			}
			return result;
		}

		std::vector<Math::Vector3df> PrimitiveGenerator::generateRect(
			const Math::Vector3df& center, const float width, const float depth,
			const int pointCount, const float noise, const uint32_t seed)
		{
			std::vector<Math::Vector3df> result;
			if (width <= 0.0f || depth <= 0.0f || pointCount <= 0) {
				return result;
			}
			result.resize(static_cast<size_t>(pointCount));

			std::mt19937 rng = makeRng(seed);
			std::normal_distribution<float> noiseDist(0.0f, noise > 0.0f ? noise : 1.0f);
			std::uniform_real_distribution<float> xDist(0.0f, width);
			std::uniform_real_distribution<float> dDist(0.0f, depth);

			for (int i = 0; i < pointCount; ++i) {
				const float xv = xDist(rng);
				const float dv = dDist(rng);
				const float noiseVal = (noise > 0.0f) ? noiseDist(rng) : 0.0f;
				result[i] = Math::Vector3df(center.x + xv, center.y + noiseVal, center.z + dv);
			}
			return result;
		}

	}
}
