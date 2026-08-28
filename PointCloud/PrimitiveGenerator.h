#pragma once

#include "../../CGLib/Math/Vector3d.h"

#include <cstdint>
#include <vector>

namespace Phantom {
	namespace PC {

		/// @brief Generates synthetic point clouds (sphere/cylinder/rectangle) for testing and
		/// procedural scene setup. Stateless: each call produces a fresh point set from its
		/// parameters, unlike the add()/execute()/get() pattern used by filters and estimators.
		class PrimitiveGenerator
		{
		public:
			/// @brief Generates points uniformly distributed on a sphere surface.
			/// @param center Sphere center.
			/// @param radius Sphere radius (must be > 0).
			/// @param pointCount Number of points to generate (must be > 0).
			/// @param noise Standard deviation of Gaussian noise added to the radius (0 = no noise).
			/// @param seed RNG seed. 0 selects a nondeterministic seed (std::random_device).
			static std::vector<Math::Vector3df> generateSphere(
				const Math::Vector3df& center, const float radius,
				const int pointCount, const float noise, const uint32_t seed = 0);

			/// @brief Generates points uniformly distributed on a cylinder's lateral surface
			/// (base centered at `center`, extruded along +Z by `height`).
			/// @param center Base center.
			/// @param radius Cylinder radius (must be > 0).
			/// @param height Cylinder height (must be > 0).
			/// @param pointCount Number of points to generate (must be > 0).
			/// @param noise Standard deviation of Gaussian noise added to the radius (0 = no noise).
			/// @param seed RNG seed. 0 selects a nondeterministic seed (std::random_device).
			static std::vector<Math::Vector3df> generateCylinder(
				const Math::Vector3df& center, const float radius, const float height,
				const int pointCount, const float noise, const uint32_t seed = 0);

			/// @brief Generates points on an XZ-plane rectangle (width along X, depth along Z),
			/// with Gaussian noise applied along Y.
			/// @param center Rectangle corner origin.
			/// @param width Rectangle extent along X (must be > 0).
			/// @param depth Rectangle extent along Z (must be > 0).
			/// @param pointCount Number of points to generate (must be > 0).
			/// @param noise Standard deviation of Gaussian noise added along Y (0 = no noise).
			/// @param seed RNG seed. 0 selects a nondeterministic seed (std::random_device).
			static std::vector<Math::Vector3df> generateRect(
				const Math::Vector3df& center, const float width, const float depth,
				const int pointCount, const float noise, const uint32_t seed = 0);
		};

	}
}
