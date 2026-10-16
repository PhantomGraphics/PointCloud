#pragma once

#include <functional>

namespace Phantom {
	namespace PC {

		/// @brief Lightweight progress/cancellation callback wrapper shared by the
		/// long-running Phantom::PC algorithms that support it (DBSCANClustering::cluster(),
		/// DistanceBasedClustering::DistanceBasedRegionGrowing(), ICPRegistration::align()).
		///
		/// A default-constructed instance (no callback set) costs a single branch per
		/// check site and never allocates, so passing the default `{}` is exactly as
		/// cheap as the pre-existing signatures without this parameter. Call sites that
		/// DO set a callback are expected to throttle invocation frequency (e.g. to
		/// O(100) calls total even for N in the millions), since on the Python-bound
		/// side each call re-acquires the GIL.
		struct ProgressReporter {
			/// @param progress Progress fraction, 0.0 (start) to 1.0 (done).
			/// @return true to continue processing; false to request the caller stop promptly.
			std::function<bool(float progress)> callback;

			bool isSet() const { return static_cast<bool>(callback); }

			/// @return true always if no callback is set (unconditional continue).
			bool report(float progress) const {
				return callback ? callback(progress) : true;
			}
		};

	}
}
