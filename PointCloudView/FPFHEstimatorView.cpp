#include "FPFHEstimatorView.h"

#include "FPFHEstimator.h"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <vector>

namespace VPC {

void FPFHEstimatorView::onImGui(World& world, int activeSceneId,
                                 const std::function<void(int)>& onResult)
{
    runButton_.setFunction([&world, activeSceneId, &onResult, this]() {
        hasResult_ = false;
        succeeded_ = false;

        auto* scene = world.findById(activeSceneId);
        if (scene == nullptr) { status_ = "Scene not found"; return; }
        if (!scene->hasNormals()) { status_ = "Scene has no normals - run Normal Estimator first"; return; }

        const auto& positions = scene->getPositions();
        const auto& normals   = scene->getNormals();

        Phantom::PC::FPFHEstimator estimator;
        for (size_t i = 0; i < positions.size(); ++i) estimator.add(positions[i], normals[i]);
        if (!estimator.estimate(static_cast<size_t>(kNeighbors_))) { status_ = "FPFH estimation failed"; return; }

        const auto histograms = estimator.getHistograms();

        Phantom::PC::FPFHEstimator::Histogram mean{};
        mean.fill(0.0f);
        for (const auto& h : histograms)
            for (size_t b = 0; b < h.size(); ++b) mean[b] += h[b] / static_cast<float>(histograms.size());

        std::vector<float> dist(histograms.size(), 0.0f);
        float maxDist = 0.0f;
        for (size_t i = 0; i < histograms.size(); ++i) {
            float d = 0.0f;
            for (size_t b = 0; b < histograms[i].size(); ++b) {
                const float diff = histograms[i][b] - mean[b];
                d += diff * diff;
            }
            dist[i] = std::sqrt(d);
            maxDist = std::max(maxDist, dist[i]);
        }
        const float norm = (maxDist > 0.0f) ? maxDist : 1.0f;

        auto* result = world.addScene("FPFHResult");
        for (size_t i = 0; i < positions.size(); ++i) {
            const float v = dist[i] / norm;
            result->add(positions[i], glm::vec3(v, 0.3f, 1.0f - v));
        }

        scene->setVisible(false);
        onResult(-1);

        hasResult_  = true;
        succeeded_  = true;
        pointCount_ = static_cast<int>(histograms.size());
        status_     = "FPFH estimation complete (33-dim descriptors)";
    });

    ImGui::SliderInt("k Neighbors", &kNeighbors_, 5, 100);

    ImGui::Separator();
    ImGui::Text("Status: %s", status_.c_str());
    if (hasResult_ && succeeded_) {
        ImGui::Text("Descriptors computed: %d", pointCount_);
    }
    runButton_.show();
}

} // namespace VPC
