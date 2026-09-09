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
		std::function<void(const GaussianPointRenderer::Params&)> onGpParamsChanged);

	void setSplatCount(size_t n) { splatCount_ = n; }
	void setParticleCount(size_t n) { particleCount_ = n; }
	void setParticleCapacity(size_t n) { particleCapacity_ = n; }
	void setFPS(float fps) { fps_ = fps; }
	void setCurrentMode(RenderMode mode) { currentMode_ = mode; }
	void setDebugSplat(const DebugSplatInfo& d) { debugSplat_ = d; }
	void setGSAvailable(bool v) { gsAvailable_ = v; }
	void setGaussianPointAvailable(bool v) { gpAvailable_ = v; }
	void setGaussianPointParams(const GaussianPointRenderer::Params& p) { gp_ = p; }
	void setGaussianPointStats(const GaussianPointRenderer::Stats& s) { gpStats_ = s; }
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
	std::function<void(const GaussianPointRenderer::Params&)> onGpParamsChanged_;

	DebugSplatInfo debugSplat_;
	bool gsAvailable_ = false;
	float splatSizeScale_ = 1000.f;

	// GaussianPoint (Phase 2/3)
	bool gpAvailable_ = false;
	GaussianPointRenderer::Params gp_;
	GaussianPointRenderer::Stats  gpStats_;
};

} // namespace GSView
