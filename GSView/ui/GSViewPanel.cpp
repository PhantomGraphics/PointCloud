#include "GSViewPanel.h"

#include "imgui.h"

namespace GSView {

void GSViewPanel::init(
	std::function<void(RenderMode)> onModeChanged,
	std::function<void(float)> onSortParamsChanged,
	std::function<void(float, int, float, int)> onPBVRParamsChanged,
	std::function<void(float)> onSplatSizeChanged,
	std::function<void(const GaussianPointRenderer::Params&)> onGpParamsChanged)
{
	onModeChanged_ = std::move(onModeChanged);
	onSortParamsChanged_ = std::move(onSortParamsChanged);
	onPBVRParamsChanged_ = std::move(onPBVRParamsChanged);
	onSplatSizeChanged_ = std::move(onSplatSizeChanged);
	onGpParamsChanged_ = std::move(onGpParamsChanged);
}

void GSViewPanel::drawEnsembleLodControls()
{
	ImGui::Spacing();
	ImGui::TextUnformatted("Ensemble LOD");
	bool lodChanged = false;
	if (ImGui::Combo("LOD Mode", &gp_.lodMode, "Off\0" "Manual\0" "Adaptive\0"))
		lodChanged = true;

	const bool manual = gp_.lodMode == 1;
	const bool adaptive = gp_.lodMode == 2;
	if (manual || adaptive) {
		lodChanged |= ImGui::SliderInt(adaptive ? "Max Ensembles/Frame" : "Ensembles/Frame",
			&gp_.ensemblesPerFrame, 1, 8);
		lodChanged |= ImGui::SliderInt("Target Ensembles", &gp_.targetEnsembles, 1, 512);
		if (adaptive) {
			lodChanged |= ImGui::SliderFloat("Frame Budget Low (ms)", &gp_.lodFrameBudgetLowMs, 4.0f, 66.0f);
			lodChanged |= ImGui::SliderFloat("Frame Budget High (ms)", &gp_.lodFrameBudgetHighMs, gp_.lodFrameBudgetLowMs, 100.0f);
		}
		if (gp_.pointBudget > 0.0f) {
			ImGui::TextColored(ImVec4(1.f, 0.75f, 0.2f, 1.f),
				"Point Budget > 0 changes density -- combined with LOD this is\n"
				"not a pure sample-count refinement.");
		}
	}
	if (lodChanged && onGpParamsChanged_) onGpParamsChanged_(gp_);

	if (manual || adaptive) {
		ImGui::Text("Ensembles this frame: %u  displayed: %u  (epoch %u)",
			gpStats_.ensemblesThisFrame, gpStats_.displayedEnsembles, gpStats_.epoch);
		if (adaptive) {
			static const char* kStateNames[] = { "Moving", "Settling", "Refining", "Converged" };
			const char* stateName = gpStats_.adaptiveState < 4 ? kStateNames[gpStats_.adaptiveState] : "?";
			ImGui::Text("State: %s  (%u frames since reset)", stateName, gpStats_.framesSinceReset);
			if (gpStats_.effectiveEnsemblesPerFrame <= 1 && gpStats_.computeMs > gp_.lodFrameBudgetHighMs) {
				ImGui::TextColored(ImVec4(1.f, 0.4f, 0.3f, 1.f),
					"GPU budget exceeded even at R=1 -- LOD alone cannot reach the target frame time.");
			}
		}
	}
}

void GSViewPanel::onImGui()
{
	ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), ImGuiCond_Once);
	// Tall enough for the GaussianPoint/PBVR3D panel plus the common part of the
	// Phase 3 Ensemble LOD block (mode combo + Manual's two sliders); Adaptive's
	// extra budget sliders and state/warning text may need the scrollbar ImGui
	// adds automatically once content exceeds this.
	ImGui::SetNextWindowSize(ImVec2(330.f, 400.f), ImGuiCond_Once);
	if (!ImGui::Begin("GSView Control")) {
		ImGui::End();
		return;
	}

	ImGui::Text("Render Mode:");
	auto radio = [&](const char* label, RenderMode m) {
		if (ImGui::RadioButton(label, currentMode_ == m) && currentMode_ != m) {
			currentMode_ = m;
			if (onModeChanged_) onModeChanged_(currentMode_);
		}
	};
	radio("Sort-Based", RenderMode::SortBased);
	ImGui::SameLine();
	if (gpAvailable_) {
		radio("PBVR 3D (exp.)", RenderMode::PBVR3DExperimental);
	} else {
		ImGui::BeginDisabled();
		ImGui::RadioButton("PBVR 3D (exp.)", false);
		ImGui::EndDisabled();
	}
	ImGui::SameLine();
	if (gpAvailable_) {
		radio("Gaussian Point", RenderMode::GaussianPoint);
	} else {
		ImGui::BeginDisabled();
		ImGui::RadioButton("Gaussian Point", false);
		ImGui::EndDisabled();
	}

	ImGui::Separator();

	if (currentMode_ == RenderMode::SortBased) {
		bool sortChanged = false;
		sortChanged |= ImGui::SliderFloat("Point Size", &sortPointSize_, 1.0f, 16.0f);
		if (sortChanged && onSortParamsChanged_) onSortParamsChanged_(sortPointSize_);

		bool sizeChanged = ImGui::SliderFloat("Splat Scale", &splatSizeScale_, 100.f, 5000.f);
		if (sizeChanged && onSplatSizeChanged_) onSplatSizeChanged_(splatSizeScale_);
	} else if (currentMode_ == RenderMode::PBVR3DExperimental) {
		bool pbvrChanged = false;
		if (ImGui::Combo("Method", &pbvr3dMethod_,
				"Proportional\0" "Extinction -log(1-o)\0" "View-conditioned\0"))
			pbvrChanged = true;
		pbvrChanged |= ImGui::SliderFloat("Density Scale", &densityScale_, 0.1f, 10.0f);
		pbvrChanged |= ImGui::SliderInt("Max Particles/Splat", &maxParticlesPerSplat_, 1, 4096);
		pbvrChanged |= ImGui::SliderFloat("Base Points x64", &pbvrParticleSize_, 1.0f, 64.0f);
		if (pbvrChanged && onPBVRParamsChanged_)
			onPBVRParamsChanged_(densityScale_, maxParticlesPerSplat_, pbvrParticleSize_, pbvr3dMethod_);
		ImGui::Spacing();
		bool zoomChanged = ImGui::Checkbox("Zoom density recalibration", &gp_.pbvrZoomRecalibration);
		if (gp_.pbvrZoomRecalibration)
			zoomChanged |= ImGui::SliderFloat("Reference pixel length", &gp_.pbvrReferencePixelLength, 0.0001f, 0.1f, "%.4f", ImGuiSliderFlags_Logarithmic);
		if (zoomChanged && onGpParamsChanged_) onGpParamsChanged_(gp_);
		ImGui::Text("expected/generated: %u / %u", gpStats_.expectedCount, gpStats_.generatedCount);
		ImGui::Text("candidates / drawn: %u / %u", gpStats_.candidateCount, gpStats_.drawnPoints);
		ImGui::Text("active / accum frames: %u / %u", gpStats_.activeSamples, gpStats_.accumFrames);
		ImGui::Text("GPU: %.2f ms", gpStats_.computeMs);
		drawEnsembleLodControls();
	} else { // GaussianPoint
		bool gpChanged = false;
		int sppIdx = gp_.sppSide - 1;
		if (ImGui::Combo("SPP", &sppIdx, "1\0" "4\0" "9\0" "16\0")) {
			gp_.sppSide = sppIdx + 1;
			gpChanged = true;
		}
		if (ImGui::Combo("Seed Mode", &gp_.seedMode, "Deterministic\0" "Frame-varying\0"))
			gpChanged = true;
		gpChanged |= ImGui::SliderFloat("Density Scale##gp", &gp_.densityScale, 0.1f, 4.0f);
		int shMax = gpStats_.shDegreeData;
		if (shMax > 0) {
			gpChanged |= ImGui::SliderInt("SH Degree", &gp_.shDegree, 0, shMax);
		} else {
			ImGui::BeginDisabled();
			int z = 0; ImGui::SliderInt("SH Degree", &z, 0, 0);
			ImGui::EndDisabled();
			ImGui::SameLine(); ImGui::TextDisabled("(DC-only data)");
		}
		if (ImGui::Combo("Tone Map", &gp_.tonemapMode, "None\0" "Reinhard\0" "ACES\0"))
			gpChanged = true;
		gpChanged |= ImGui::SliderFloat("Gamma##gp", &gp_.gamma, 0.5f, 2.4f);
		{
			float budgetM = gp_.pointBudget / 1.0e6f;
			if (ImGui::SliderFloat("Point Budget (M, 0=off)", &budgetM, 0.0f, 4.0f)) {
				gp_.pointBudget = budgetM * 1.0e6f;
				gpChanged = true;
			}
		}
		if (gpChanged && onGpParamsChanged_)
			onGpParamsChanged_(gp_);
		ImGui::Spacing();
		ImGui::Text("expected/generated: %u / %u", gpStats_.expectedCount, gpStats_.generatedCount);
		ImGui::Text("active / drawn: %u / %u", gpStats_.activeSamples, gpStats_.drawnPoints);
		ImGui::Text("accumulated frames: %u", gpStats_.accumFrames);
		ImGui::Text("GPU: %.2f ms  (splat %.2f  resolve %.2f)",
			gpStats_.computeMs, gpStats_.splatDepthMs + gpStats_.splatColorMs, gpStats_.resolveMs);
		drawEnsembleLodControls();
	}

	ImGui::Separator();
	ImGui::Text("Loaded splats:    %zu", splatCount_);
	ImGui::Text("Particles (PBVR): %zu / %zu cap", particleCount_, particleCapacity_);
	ImGui::Text("FPS: %.1f", fps_);

	ImGui::End();

	// ---- Debug window (splat #0) ----------------------------------------
	ImGui::SetNextWindowPos(ImVec2(10.f, 445.f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(330.f, 265.f), ImGuiCond_Once);
	if (ImGui::Begin("GS Debug (splat #0)")) {
		ImGui::TextColored(
			gsAvailable_ ? ImVec4(0.2f,1.f,0.2f,1.f) : ImVec4(1.f,0.3f,0.3f,1.f),
			gsAvailable_ ? "GS Pipeline: OK" : "GS Pipeline: UNAVAILABLE (fallback to Point)");
		ImGui::Separator();
		if (!debugSplat_.valid) {
			ImGui::TextDisabled("No splat loaded.");
		} else {
			ImGui::Text("log_scale: (%.3f, %.3f, %.3f)",
				debugSplat_.rawScale[0], debugSplat_.rawScale[1], debugSplat_.rawScale[2]);
			ImGui::Text("exp_scale: (%.5f, %.5f, %.5f)",
				debugSplat_.sx, debugSplat_.sy, debugSplat_.sz);
			ImGui::Text("maxScale : %.5f", debugSplat_.maxScale);
			ImGui::Text("quat(w,x,y,z): (%.3f, %.3f, %.3f, %.3f)",
				debugSplat_.rawQuat[0], debugSplat_.rawQuat[1],
				debugSplat_.rawQuat[2], debugSplat_.rawQuat[3]);
			ImGui::Text("pointSize: %.2f", debugSplat_.pointSize);
			ImGui::Separator();
			ImGui::Text("vMatrix cols (maxScale*S^-1*R^T):");
			ImGui::Text("  col0: (%.4f, %.4f, %.4f)",
				debugSplat_.covRow0.x, debugSplat_.covRow0.y, debugSplat_.covRow0.z);
			ImGui::Text("  col1: (%.4f, %.4f, %.4f)",
				debugSplat_.covRow1.x, debugSplat_.covRow1.y, debugSplat_.covRow1.z);
			ImGui::Text("  col2: (%.4f, %.4f, %.4f)",
				debugSplat_.covRow2.x, debugSplat_.covRow2.y, debugSplat_.covRow2.z);
			ImGui::Separator();
			// Singular values: |col_j| = maxScale/s_j (should be >=1)
			const float sv0 = glm::length(debugSplat_.covRow0);
			const float sv1 = glm::length(debugSplat_.covRow1);
			const float sv2 = glm::length(debugSplat_.covRow2);
			ImGui::Text("col lengths (=maxScale/s_j):");
			ImGui::Text("  %.4f  %.4f  %.4f", sv0, sv1, sv2);
			ImGui::TextColored(
				(sv0 >= 0.9f && sv1 >= 0.9f && sv2 >= 0.9f)
					? ImVec4(0.2f,1.f,0.2f,1.f)
					: ImVec4(1.f,0.4f,0.2f,1.f),
				(sv0 >= 0.9f && sv1 >= 0.9f && sv2 >= 0.9f)
					? "OK (max axis fills sprite)"
					: "WARN: col length < 1");
		}
	}
	ImGui::End();
}

} // namespace GSView
