#include "../include/VkPointRenderer.h"

#include "../../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <algorithm>
#include <array>

namespace VKR {

VkVertexInputBindingDescription VkPointRenderer::Vertex::getBindingDescription() {
    VkVertexInputBindingDescription bd{};
    bd.binding = 0;
    bd.stride = sizeof(Vertex);
    bd.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bd;
}

std::array<VkVertexInputAttributeDescription, 2> VkPointRenderer::Vertex::getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 2> attrs{};
    attrs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos) };
    attrs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color) };
    return attrs;
}

void VkPointRenderer::setActiveScene(int id) {
    activeSceneId_ = id;
    notifyActiveChanged();
}

void VkPointRenderer::notifySceneChanged() {
    rebuildVertexList();
    rebuildGSSplatListFromActiveScene();
    if (showNormals_) normalsDirty_ = true;
}

void VkPointRenderer::notifyActiveChanged() {
    rebuildGSSplatListFromActiveScene();
    if (showNormals_) normalsDirty_ = true;
}

void VkPointRenderer::onInit(const Phantom::VKG::VulkanContext& ctx,
                             const Phantom::VKG::VulkanCommandPool& pool,
                             VkRenderPass renderPass,
                             uint32_t framesInFlight,
                             Shaders shaders)
{
    ctx_ = &ctx;
    pool_ = &pool;

    VkPointCloudPipelineConfig cfg;
    cfg.vertSpv = std::move(shaders.pointVert);
    cfg.fragSpv = std::move(shaders.pointFrag);
    cfg.bindingDesc = Vertex::getBindingDescription();
    auto attrs = Vertex::getAttributeDescriptions();
    cfg.attrDescs = { attrs[0], attrs[1] };
    cfg.framesInFlight = framesInFlight;
    pipeline_.create(ctx, renderPass, cfg);

    {
        VkGSPointRendererConfig gsCfg;
        gsCfg.vertSpv = std::move(shaders.gsVert);
        gsCfg.fragSpv = std::move(shaders.gsFrag);
        gsCfg.compSpv = std::move(shaders.gsComp);
        gsCfg.framesInFlight = framesInFlight;
        gsCfg.samples = VK_SAMPLE_COUNT_1_BIT;
        gsRenderer_ = std::make_unique<VkGSPointRenderer>();
        if (gsRenderer_->create(ctx, pool, renderPass, gsCfg)) {
            gsAvailable_ = true;
        } else {
            gsRenderer_.reset();
            gsAvailable_ = false;
            renderMode_ = PointRenderMode::Point;
        }
    }

    Phantom::VKG::VkLineRenderer::Config lineCfg;
    lineCfg.vertSpv = std::move(shaders.lineVert);
    lineCfg.fragSpv = std::move(shaders.lineFrag);
    lineRenderer_ = std::make_unique<Phantom::VKG::VkLineRenderer>(std::move(lineCfg));
    lineRenderer_->create(ctx, pool, renderPass, framesInFlight);

    notifySceneChanged();
    uploadVertexBuffer();
    if (gsAvailable_ && gsDirty_ && gsRenderer_) {
        gsRenderer_->upload(*ctx_, pendingSplats_);
        gsDirty_ = false;
    }
    pointsDirty_ = false;
}

void VkPointRenderer::onUpdate(uint32_t frameIndex, const glm::mat4& mvp, const glm::vec3& eye) {
    if (pointsDirty_) {
        vkDeviceWaitIdle(ctx_->getDevice());
        vertexBuffer_.destroy(ctx_->getDevice());
        uploadVertexBuffer();
        pointsDirty_ = false;
    }

    if (gsAvailable_ && gsDirty_ && gsRenderer_) {
        vkDeviceWaitIdle(ctx_->getDevice());
        gsRenderer_->upload(*ctx_, pendingSplats_);
        gsDirty_ = false;
    }

    currentMVP_ = mvp;
    pipeline_.updateUBO(frameIndex, currentMVP_, pointSize_);
    if (gsAvailable_ && gsRenderer_)
        gsRenderer_->updateMVP(frameIndex, currentMVP_);

    if (gsAvailable_ && renderMode_ == PointRenderMode::GaussianSplatting && gsRenderer_)
        gsRenderer_->sortByView(eye);

    if (showNormals_ && lineRenderer_) {
        if (normalsDirty_) {
            buildNormalLines();
            normalsDirty_ = false;
        }
        lineRenderer_->updateMVP(frameIndex, currentMVP_);
    }
}

void VkPointRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex) {
    if (gsAvailable_ && renderMode_ == PointRenderMode::GaussianSplatting && gsRenderer_ && gsRenderer_->hasData()) {
        gsRenderer_->render(cmd, frameIndex);
    } else if (pointCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());

        VkBuffer vbufs[] = { vertexBuffer_.get() };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(cmd, 0, 1, vbufs, offsets);

        VkDescriptorSet ds = pipeline_.getDescriptorSet(frameIndex);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipeline_.getLayout(), 0, 1,
                                &ds, 0, nullptr);

        vkCmdDraw(cmd, pointCount_, 1, 0, 0);
    }

    if (showNormals_ && lineRenderer_) {
        const auto* active = scene_ ? scene_->find(activeSceneId_) : nullptr;
        if (active && !active->normals.empty())
            lineRenderer_->render(cmd, frameIndex);
    }
}

void VkPointRenderer::onCleanup(VkDevice device) {
    if (lineRenderer_) { lineRenderer_->destroy(device); lineRenderer_.reset(); }
    if (gsRenderer_) { gsRenderer_->destroy(device); gsRenderer_.reset(); }
    gsAvailable_ = false;
    vertexBuffer_.destroy(device);
    pipeline_.destroy(device);
    ctx_ = nullptr;
    pool_ = nullptr;
}

void VkPointRenderer::rebuildVertexList() {
    pendingPoints_.clear();
    if (!scene_) return;

    for (int id : scene_->allIds()) {
        const auto* s = scene_->find(id);
        if (!s || !s->visible) continue;
        const size_t n = std::min(s->positions.size(), s->colors.size());
        for (size_t i = 0; i < n; ++i)
            pendingPoints_.push_back({ s->positions[i], s->colors[i] });
    }

    currentPoints_ = pendingPoints_;
    pointsDirty_ = true;
}

void VkPointRenderer::rebuildGSSplatListFromActiveScene() {
    pendingSplats_.clear();
    const auto* active = scene_ ? scene_->find(activeSceneId_) : nullptr;
    if (active && !active->gsSplats.empty())
        pendingSplats_ = active->gsSplats;
    gsDirty_ = true;
}

void VkPointRenderer::uploadVertexBuffer() {
    if (pendingPoints_.empty()) { pointCount_ = 0; return; }
    pointCount_ = static_cast<uint32_t>(pendingPoints_.size());
    vertexBuffer_.create(*ctx_, *pool_,
                         sizeof(Vertex) * pendingPoints_.size(),
                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                         pendingPoints_.data());
}

void VkPointRenderer::buildNormalLines() {
    Phantom::VKG::VkLineRenderer::Buffer buf;
    buf.projectionMatrix = glm::mat4(1.f);
    buf.modelViewMatrix = glm::mat4(1.f);

    const auto* active = scene_ ? scene_->find(activeSceneId_) : nullptr;
    if (active && !active->normals.empty()) {
        const size_t n = std::min(active->positions.size(), active->normals.size());
        buf.positions.reserve(n * 6);
        buf.colors.reserve(n * 8);
        buf.indices.reserve(n * 2);

        for (size_t i = 0; i < n; ++i) {
            const glm::vec3& p = active->positions[i];
            const glm::vec3 q = p + active->normals[i] * normalLength_;
            const auto base = static_cast<uint32_t>(i * 2);
            buf.positions.insert(buf.positions.end(), { p.x, p.y, p.z, q.x, q.y, q.z });
            buf.colors.insert(buf.colors.end(), {
                0.3f, 0.9f, 1.0f, 1.0f,
                1.0f, 1.0f, 0.2f, 1.0f
            });
            buf.indices.push_back(base);
            buf.indices.push_back(base + 1);
        }
    }

    if (lineRenderer_) {
        vkDeviceWaitIdle(ctx_->getDevice());
        lineRenderer_->upload(*ctx_, *pool_, buf);
    }
}

} // namespace VKR
