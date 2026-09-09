#pragma once

#include "../GSViewRenderer.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace GSView {

class GSViewPanel {
public:
	void init(
		std::function<void(RenderMode)> onModeChanged,
		std::function<void(float)> onSortParamsChanged,
		std::function<void(float, int, float)> onPBVRParamsChanged,
		std::function<void(float)> onSplatSizeChanged,
		std::function<void(int, int, float)> onGpParamsChanged);

	void setSplatCount(size_t n) { splatCount_ = n; }
	void setParticleCount(size_t n) { particleCount_ = n; }
	void setParticleCapacity(size_t n) { particleCapacity_ = n; }
	void setFPS(float fps) { fps_ = fps; }
	void setCurrentMode(RenderMode mode) { currentMode_ = mode; }
	void setDebugSplat(const DebugSplatInfo& d) { debugSplat_ = d; }
	void setGSAvailable(bool v) { gsAvailable_ = v; }
	void setGaussianPointAvailable(bool v) { gpAvailable_ = v; }
	void setGaussianPointStats(uint32_t expected, uint32_t generated, uint32_t active, uint32_t drawn) {
		gpExpected_ = expected; gpGenerated_ = generated; gpActive_ = active; gpDrawn_ = drawn;
	}
	void setSplatSizeScale(float v) { splatSizeScale_ = v; }
	float getSplatSizeScale() const { return splatSizeScale_; }

	float getSortPointSize() const { return sortPointSize_; }
	float getDensityScale() const { return densityScale_; }
	int getMaxParticlesPerSplat() const { return maxParticlesPerSplat_; }
	float getPbvrParticleSize() const { return pbvrParticleSize_; }

	void onImGui();

private:
	RenderMode currentMode_ = RenderMode::SortBased;
	float sortPointSize_ = 2.0f;
	float densityScale_ = 1.0f;
	int maxParticlesPerSplat_ = 8;
	float pbvrParticleSize_ = 4.0f;
	size_t splatCount_ = 0;
	size_t particleCount_ = 0;
	size_t particleCapacity_ = 0;
	float fps_ = 0.0f;

	std::function<void(RenderMode)> onModeChanged_;
	std::function<void(float)> onSortParamsChanged_;
	std::function<void(float, int, float)> onPBVRParamsChanged_;
	std::function<void(float)> onSplatSizeChanged_;
	std::function<void(int, int, float)> onGpParamsChanged_;

	DebugSplatInfo debugSplat_;
	bool gsAvailable_ = false;
	float splatSizeScale_ = 1000.f;

	// GaussianPoint (Phase 2)
	bool gpAvailable_ = false;
	int gpSppSide_ = 2;
	int gpSeedMode_ = 1;   // 0 deterministic, 1 frame-varying
	float gpDensityScale_ = 1.0f;
	uint32_t gpExpected_ = 0, gpGenerated_ = 0, gpActive_ = 0, gpDrawn_ = 0;
};

} // namespace GSView
