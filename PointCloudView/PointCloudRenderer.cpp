#include "PointCloudRenderer.h"

#include "imgui.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace VPC {

// ============================================================
//  Vertex
// ============================================================

VkVertexInputBindingDescription Vertex::getBindingDescription() {
    VkVertexInputBindingDescription bd{};
    bd.binding = 0;
    bd.stride = sizeof(Vertex);
    bd.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bd;
}

std::array<VkVertexInputAttributeDescription, 2> Vertex::getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 2> attrs{};
    attrs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos) };
    attrs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color) };
    return attrs;
}

// ============================================================
//  Public API
// ============================================================

void PointCloudRenderer::setActiveSceneId(int id) {
    activeSceneId_ = id;
    stateDirty_ = true;
}

void PointCloudRenderer::setRenderMode(RenderMode mode) {
    renderMode_ = mode;
    stateDirty_ = true;
}

void PointCloudRenderer::handleMouseButton(bool leftPressed) {
    isDragging_ = leftPressed;
}

void PointCloudRenderer::handleMouseMove(double x, double y) {
    if (isDragging_) {
        const float dx = static_cast<float>(x - lastX_) * 0.005f;
        const float dy = static_cast<float>(y - lastY_) * 0.005f;
        camPhi_ -= dx;
        camTheta_ = std::max(0.05f, std::min(3.09f, camTheta_ + dy));
    }
    lastX_ = x;
    lastY_ = y;
}

void PointCloudRenderer::handleScroll(double dy) {
    camDist_ = std::max(0.1f, camDist_ - static_cast<float>(dy) * 0.2f);
}

void PointCloudRenderer::notifyWorldChanged() {
    syncScene();
    syncPolygons();
    stateDirty_ = true;
    rendererCore_.notifySceneChanged();
}

void PointCloudRenderer::notifySceneSelectionChanged() {
    syncScene();
    stateDirty_ = true;
    rendererCore_.notifyActiveChanged();
}

// ============================================================
//  IVkSubRenderer
// ============================================================

void PointCloudRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                                      const Phantom::VKG::VulkanCommandPool& pool,
                                      VkRenderPass renderPass,
                                      uint32_t framesInFlight)
{
    ctx_ = &ctx;
    pool_ = &pool;

    if (world_ && world_->isEmpty()) {
        auto* scene = world_->addScene("demo_sphere");
        setActiveSceneId(scene->getId());
        srand(42);
        auto rf = []() { return static_cast<float>(rand()) / RAND_MAX; };
        for (int i = 0; i < 50000; ++i) {
            const float u = rf(), v = rf();
            const float theta = 2.f * 3.14159f * u;
            const float phi = acosf(1.f - 2.f * v);
            const float r = 1.0f + 0.05f * (rf() - 0.5f);
            scene->add({ r * sinf(phi) * cosf(theta),
                         r * sinf(phi) * sinf(theta),
                         r * cosf(phi) },
                       { rf(), rf(), rf() });
        }
    }

    syncScene();
    syncPolygons();

    rendererCore_.setScene(&scene_);
    rendererCore_.setActiveScene(activeSceneId_);
    rendererCore_.setRenderMode(
        renderMode_ == RenderMode::Point
        ? VKR::PointRenderMode::Point
        : VKR::PointRenderMode::GaussianSplatting);
    rendererCore_.setShowNormals(showNormals_);
    rendererCore_.setNormalLength(normalLength_);
    {
        VKR::VkPointRenderer::Shaders s;
        s.pointVert = std::move(shaders_.pointVert);
        s.pointFrag = std::move(shaders_.pointFrag);
        s.gsVert    = std::move(shaders_.gsVert);
        s.gsFrag    = std::move(shaders_.gsFrag);
        s.gsComp    = std::move(shaders_.gsComp);
        s.lineVert  = std::move(shaders_.lineVert);
        s.lineFrag  = std::move(shaders_.lineFrag);
        rendererCore_.onInit(ctx, pool, renderPass, framesInFlight, std::move(s));
    }

    gsAvailable_ = rendererCore_.isGaussianSplattingAvailable();
    if (!gsAvailable_) setRenderMode(RenderMode::Point);

    if (!shaders_.triVert.empty() && !shaders_.triFrag.empty()) {
        Phantom::VKG::VkTriangleRenderer::Config triCfg;
        triCfg.vertSpv     = std::move(shaders_.triVert);
        triCfg.fragSpv     = std::move(shaders_.triFrag);
        triCfg.blendEnable = true;
        triCfg.cullBack    = false;
        triRenderer_ = std::make_unique<Phantom::VKG::VkTriangleRenderer>(std::move(triCfg));
        triRenderer_->create(ctx, pool, renderPass, framesInFlight);
        if (!pendingPolygonBuf_.indices.empty())
            triRenderer_->upload(ctx, pool, pendingPolygonBuf_);
        polygonDirty_ = false;
    }
}

void PointCloudRenderer::onUpdate(uint32_t frameIndex) {
    if (stateDirty_) {
        rendererCore_.setActiveScene(activeSceneId_);
        rendererCore_.setRenderMode(
            renderMode_ == RenderMode::Point
            ? VKR::PointRenderMode::Point
            : VKR::PointRenderMode::GaussianSplatting);
        rendererCore_.setShowNormals(showNormals_);
        rendererCore_.setNormalLength(normalLength_);
        stateDirty_ = false;
    }

    const glm::mat4 mvp = computeMVP();
    const glm::vec3 eye = computeEye();
    rendererCore_.onUpdate(frameIndex, mvp, eye);

    gsAvailable_ = rendererCore_.isGaussianSplattingAvailable();
    pointCount_  = rendererCore_.getPointCount();

    if (triRenderer_) {
        if (polygonDirty_) {
            vkDeviceWaitIdle(ctx_->getDevice());
            triRenderer_->upload(*ctx_, *pool_, pendingPolygonBuf_);
            polygonDirty_ = false;
        }
        triRenderer_->updateMVP(frameIndex, mvp);
    }
}

void PointCloudRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex) {
    rendererCore_.onRender(cmd, frameIndex);
    if (triRenderer_ && triRenderer_->isValid())
        triRenderer_->render(cmd, frameIndex);
}

void PointCloudRenderer::onCleanup(VkDevice device) {
    if (triRenderer_) { triRenderer_->destroy(device); triRenderer_.reset(); }
    rendererCore_.onCleanup(device);
    gsAvailable_ = false;
}

void PointCloudRenderer::drawImGuiControls() {
    if (!gsAvailable_ && renderMode_ == RenderMode::GaussianSplatting)
        setRenderMode(RenderMode::Point);

    int renderMode = (renderMode_ == RenderMode::Point) ? 0 : 1;
    if (ImGui::RadioButton("Render Point", renderMode == 0)) setRenderMode(RenderMode::Point);
    ImGui::SameLine();
    if (!gsAvailable_) ImGui::BeginDisabled();
    if (ImGui::RadioButton("Render Gaussian", renderMode == 1)) setRenderMode(RenderMode::GaussianSplatting);
    if (!gsAvailable_) {
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("(shader not found)");
    }

    auto* activeScene = world_ ? world_->findById(activeSceneId_) : nullptr;
    const bool hasNormals = activeScene && activeScene->hasNormals();
    if (!hasNormals) ImGui::BeginDisabled();
    if (ImGui::Checkbox("Show Normals", &showNormals_))
        stateDirty_ = true;
    if (showNormals_ && hasNormals) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.f);
        if (ImGui::SliderFloat("##NLen", &normalLength_, 0.001f, 0.5f, "Len=%.3f"))
            stateDirty_ = true;
    }
    if (!hasNormals) ImGui::EndDisabled();

    if (ImGui::CollapsingHeader("Camera")) {
        ImGui::SliderFloat("Theta", &camTheta_, 0.05f, 3.09f);
        ImGui::SliderFloat("Phi", &camPhi_, -3.14159f, 3.14159f);
        ImGui::SliderFloat("Dist", &camDist_, 0.1f, 50.0f);
    }
}

void PointCloudRenderer::syncScene() {
    scene_.clear();
    currentPoints_.clear();
    pointCount_ = 0;

    if (!world_) return;

    for (const auto& srcScene : world_->getScenes()) {
        const int id = scene_.add(srcScene->getName(), srcScene->getId());

        auto positions = srcScene->getPositions();
        auto colors = srcScene->getColors();
        scene_.addPoints(id, positions, colors);
        scene_.setNormals(id, srcScene->getNormals());

        std::vector<VKR::GSSplat> splats;
        const auto& srcSplats = srcScene->getGSSplats();
        splats.reserve(srcSplats.size());
        for (const auto& s : srcSplats) {
            VKR::GSSplat d{};
            d.centerSize = s.centerSize;
            d.covRow0 = s.covRow0;
            d.covRow1 = s.covRow1;
            d.covRow2 = s.covRow2;
            d.color = s.color;
            splats.push_back(d);
        }
        scene_.setGSSplats(id, splats);
        scene_.setVisible(id, srcScene->isVisible());

        if (srcScene->isVisible()) {
            const size_t n = std::min(positions.size(), colors.size());
            for (size_t i = 0; i < n; ++i)
                currentPoints_.push_back({ positions[i], colors[i] });
        }
    }

    pointCount_ = static_cast<uint32_t>(currentPoints_.size());
}

void PointCloudRenderer::syncPolygons() {
    pendingPolygonBuf_ = Phantom::VKG::VkTriangleRenderer::Buffer{};

    if (!world_) return;

    uint32_t vertexOffset = 0;
    for (const auto& mesh : world_->getPolygons()) {
        if (!mesh.visible || mesh.indices.empty()) continue;

        pendingPolygonBuf_.positions.insert(pendingPolygonBuf_.positions.end(),
            mesh.positions.begin(), mesh.positions.end());
        pendingPolygonBuf_.colors.insert(pendingPolygonBuf_.colors.end(),
            mesh.colors.begin(), mesh.colors.end());

        for (uint32_t idx : mesh.indices)
            pendingPolygonBuf_.indices.push_back(idx + vertexOffset);

        vertexOffset += static_cast<uint32_t>(mesh.positions.size() / 3);
    }

    polygonDirty_ = true;
}

glm::mat4 PointCloudRenderer::computeMVP() const {
    const glm::vec3 eye = computeEye();
    const glm::mat4 view = glm::lookAt(eye, camTarget_, glm::vec3(0.f, 1.f, 0.f));

    const float aspect = (extent_.height > 0)
        ? static_cast<float>(extent_.width) / static_cast<float>(extent_.height)
        : 1.f;
    glm::mat4 proj = glm::perspective(glm::radians(45.f), aspect, 0.01f, 100.f);
    proj[1][1] *= -1.f;
    return proj * view;
}

glm::vec3 PointCloudRenderer::computeEye() const {
    const float x = camDist_ * sinf(camTheta_) * cosf(camPhi_);
    const float y = camDist_ * cosf(camTheta_);
    const float z = camDist_ * sinf(camTheta_) * sinf(camPhi_);
    return camTarget_ + glm::vec3(x, y, z);
}

} // namespace VPC
