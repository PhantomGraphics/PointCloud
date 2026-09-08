#include "GSViewPanel.h"

#include "imgui.h"

namespace GSView {

void GSViewPanel::init(
	std::function<void(RenderMode)> onModeChanged,
	std::function<void(float)> onSortParamsChanged,
	std::function<void(float, int, float)> onPBVRParamsChanged,
	std::function<void(float)> onSplatSizeChanged)
{
	onModeChanged_ = std::move(onModeChanged);
	onSortParamsChanged_ = std::move(onSortParamsChanged);
	onPBVRParamsChanged_ = std::move(onPBVRParamsChanged);
	onSplatSizeChanged_ = std::move(onSplatSizeChanged);
}

void GSViewPanel::onImGui()
{
	ImGui::SetNextWindowPos(ImVec2(10.f, 35.f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(320.f, 340.f), ImGuiCond_Once);
	if (!ImGui::Begin("GSView Control")) {
		ImGui::End();
		return;
	}

	ImGui::Text("Render Mode:");
	const int modeIdx = (currentMode_ == RenderMode::SortBased) ? 0 : 1;
	bool changed = false;
	if (ImGui::RadioButton("Sort-Based", modeIdx == 0)) {
		if (currentMode_ != RenderMode::SortBased) {
			currentMode_ = RenderMode::SortBased;
			changed = true;
		}
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("PBVR 3D (exp.)", modeIdx == 1)) {
		if (currentMode_ != RenderMode::PBVR3DExperimental) {
			currentMode_ = RenderMode::PBVR3DExperimental;
			changed = true;
		}
	}
	if (changed && onModeChanged_) onModeChanged_(currentMode_);

	ImGui::Separator();

	if (currentMode_ == RenderMode::SortBased) {
		bool sortChanged = false;
		sortChanged |= ImGui::SliderFloat("Point Size", &sortPointSize_, 1.0f, 16.0f);
		if (sortChanged && onSortParamsChanged_) onSortParamsChanged_(sortPointSize_);

		bool sizeChanged = ImGui::SliderFloat("Splat Scale", &splatSizeScale_, 100.f, 5000.f);
		if (sizeChanged && onSplatSizeChanged_) onSplatSizeChanged_(splatSizeScale_);
	} else {
		bool pbvrChanged = false;
		pbvrChanged |= ImGui::SliderFloat("Density Scale", &densityScale_, 0.1f, 10.0f);
		pbvrChanged |= ImGui::SliderInt("Max Particles/Splat", &maxParticlesPerSplat_, 1, 32);
		pbvrChanged |= ImGui::SliderFloat("Particle Size", &pbvrParticleSize_, 1.0f, 16.0f);
		if (pbvrChanged && onPBVRParamsChanged_)
			onPBVRParamsChanged_(densityScale_, maxParticlesPerSplat_, pbvrParticleSize_);
	}

	ImGui::Separator();
	ImGui::Text("Loaded splats:    %zu", splatCount_);
	ImGui::Text("Particles (PBVR): %zu / %zu cap", particleCount_, particleCapacity_);
	ImGui::Text("FPS: %.1f", fps_);

	ImGui::End();

	// ---- Debug window (splat #0) ----------------------------------------
	ImGui::SetNextWindowPos(ImVec2(10.f, 390.f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(320.f, 300.f), ImGuiCond_Once);
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
