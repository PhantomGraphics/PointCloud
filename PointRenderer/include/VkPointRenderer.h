#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "VkPointCloudPipeline.h"
#include "VkGSPointRenderer.h"
#include "VkPointScene.h"

#include "../../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../../CGLib/Renderer/VkRenderer/VkLineRenderer.h"

#include <array>
#include <memory>
#include <vector>

namespace Phantom::VKG {
class VulkanContext;
class VulkanCommandPool;
}

namespace VKR {

enum class PointRenderMode {
    Point,
    GaussianSplatting
};

class VkPointRenderer {
public:
    struct Vertex {
        glm::vec3 pos;
        glm::vec3 color;

        static VkVertexInputBindingDescription getBindingDescription();
        static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions();
    };

    struct Shaders {
        std::vector<uint32_t> pointVert;
        std::vector<uint32_t> pointFrag;
        std::vector<uint32_t> gsVert;
        std::vector<uint32_t> gsFrag;
        std::vector<uint32_t> gsComp;
        std::vector<uint32_t> lineVert;
        std::vector<uint32_t> lineFrag;
    };

    void onInit(const Phantom::VKG::VulkanContext& ctx,
                const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass,
                uint32_t framesInFlight,
                Shaders shaders);
    void onUpdate(uint32_t frameIndex, const glm::mat4& mvp, const glm::vec3& eye);
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex);
    void onCleanup(VkDevice device);

    void setScene(VkPointScene* scene) { scene_ = scene; }
    void setActiveScene(int id);
    void notifySceneChanged();
    void notifyActiveChanged();

    void setRenderMode(PointRenderMode mode) { renderMode_ = mode; }
    void setShowNormals(bool show) { showNormals_ = show; if (showNormals_) normalsDirty_ = true; }
    void setPointSize(float pointSize) { pointSize_ = pointSize; }
    void setNormalLength(float length) { normalLength_ = length; normalsDirty_ = true; }

    PointRenderMode getRenderMode() const { return renderMode_; }
    bool isGaussianSplattingAvailable() const { return gsAvailable_; }
    uint32_t getPointCount() const { return pointCount_; }
    const std::vector<Vertex>& getCurrentPoints() const { return currentPoints_; }

private:
    const Phantom::VKG::VulkanContext* ctx_ = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;
    VkPointScene* scene_ = nullptr;
    int activeSceneId_ = -1;

    std::vector<Vertex> pendingPoints_;
    std::vector<Vertex> currentPoints_;
    uint32_t pointCount_ = 0;
    bool pointsDirty_ = false;

    VkPointCloudPipeline pipeline_;
    Phantom::VKG::VulkanBuffer vertexBuffer_;

    std::unique_ptr<VkGSPointRenderer> gsRenderer_;
    std::vector<GSSplat> pendingSplats_;
    bool gsAvailable_ = false;
    bool gsDirty_ = false;

    PointRenderMode renderMode_ = PointRenderMode::Point;
    bool showNormals_ = false;
    float pointSize_ = 1.0f;
    float normalLength_ = 0.05f;
    bool normalsDirty_ = false;

    std::unique_ptr<Phantom::VKG::VkLineRenderer> lineRenderer_;
    glm::mat4 currentMVP_{1.f};

    void rebuildVertexList();
    void rebuildGSSplatListFromActiveScene();
    void uploadVertexBuffer();
    void buildNormalLines();
};

} // namespace VKR
