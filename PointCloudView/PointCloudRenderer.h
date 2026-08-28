#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../PointRenderer/include/VkPointRenderer.h"
#include "../../CGLib/Renderer/VkRenderer/VkTriangleRenderer.h"
#include "World.h"

#include <array>
#include <memory>
#include <vector>

namespace VPC {

// GPU vertex layout: position (location 0) + color (location 1).
struct Vertex {
    glm::vec3 pos;
    glm::vec3 color;

    static VkVertexInputBindingDescription                   getBindingDescription();
    static std::array<VkVertexInputAttributeDescription, 2>  getAttributeDescriptions();
};

// IVkSubRenderer implementation for point cloud rendering.
// Owns camera state, vertex buffer, pipeline, and optional normal-line renderer.
//
// Setup before VkAppBase::run():
//   renderer.setWorld(&world);           // share the scene container
//   app.add(&renderer);                  // register lifecycle hooks
//
// Mouse events forwarded from VkAppBase window callbacks:
//   renderer.handleMouseButton(pressed);
//   renderer.handleMouseMove(x, y);
//   renderer.handleScroll(dy);
class PointCloudRenderer : public ::VKG::IVkSubRenderer {
public:
    enum class RenderMode {
        Point,
        GaussianSplatting
    };

    struct Shaders {
        std::vector<uint32_t> pointVert;
        std::vector<uint32_t> pointFrag;
        std::vector<uint32_t> gsVert;
        std::vector<uint32_t> gsFrag;
        std::vector<uint32_t> gsComp;
        std::vector<uint32_t> lineVert;
        std::vector<uint32_t> lineFrag;
        std::vector<uint32_t> triVert;
        std::vector<uint32_t> triFrag;
    };

    void setWorld(World* world) { world_ = world; }
    void setShaders(Shaders shaders) { shaders_ = std::move(shaders); }

    int  getActiveSceneId()  const { return activeSceneId_; }
    void setActiveSceneId(int id);
    void setRenderMode(RenderMode mode);
    RenderMode getRenderMode() const { return renderMode_; }
    RenderMode* getRenderModePtr() { return &renderMode_; }
    bool isGaussianSplattingAvailable() const { return gsAvailable_; }
    int* getActiveSceneIdPtr()     { return &activeSceneId_; }

    void handleMouseButton(bool leftPressed);
    void handleMouseMove(double x, double y);
    void handleScroll(double dy);

    // Trigger a full vertex list rebuild from World (e.g. after file load or processing).
    void notifyWorldChanged();

    // Notify that only the active scene selection changed (for normal-line invalidation).
    void notifySceneSelectionChanged();

    // Update the swapchain extent used for the projection aspect ratio.
    // Call from VkAppBase::onSwapChainCreated() and onInit().
    void setExtent(VkExtent2D ext) { extent_ = ext; }

    uint32_t getPointCount() const { return pointCount_; }
    const std::vector<Vertex>& getCurrentPoints() const { return currentPoints_; }

    // Called by VulkanPointCloudMenuPanel::onImGui() inside the "Control" window.
    void drawImGuiControls();

    // IVkSubRenderer
    void onInit(Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool,
                VkRenderPass renderPass, uint32_t framesInFlight) override;
    void onUpdate(uint32_t frameIndex) override;
    void onRender(VkCommandBuffer cmd, uint32_t frameIndex) override;
    void onCleanup(VkDevice device) override;

private:
    // Camera (spherical coordinates)
    float     camTheta_  = 0.4f;
    float     camPhi_    = 0.5f;
    float     camDist_   = 3.0f;
    glm::vec3 camTarget_ = {0.f, 0.f, 0.f};
    double    lastX_     = 0.;
    double    lastY_     = 0.;
    bool      isDragging_ = false;

    // Scene
    World* world_         = nullptr;
    int    activeSceneId_ = -1;

    // CPU cache for export
    std::vector<Vertex> currentPoints_;
    uint32_t pointCount_  = 0;

    // Vulkan resources (non-owning refs set in onInit)
    const Phantom::VKG::VulkanContext*     ctx_   = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_  = nullptr;
    Shaders                       shaders_;
    VKR::VkPointScene             scene_;
    VKR::VkPointRenderer          rendererCore_;
    RenderMode renderMode_ = RenderMode::Point;
    bool gsAvailable_ = false;

    // Swapchain extent (updated via setExtent)
    VkExtent2D extent_ = { 1280, 720 };

    // Dirty flag: set when active scene, render mode, or normal settings change.
    // Cleared in onUpdate() after pushing the new state to rendererCore_.
    bool stateDirty_ = true;

    // Normal visualization
    bool  showNormals_  = false;
    float normalLength_ = 0.05f;

    // Polygon overlay (plane/cylinder detection results)
    std::unique_ptr<Phantom::VKG::VkTriangleRenderer> triRenderer_;
    Phantom::VKG::VkTriangleRenderer::Buffer pendingPolygonBuf_;
    bool polygonDirty_ = false;

    void syncScene();
    void syncPolygons();
    glm::mat4 computeMVP() const;
    glm::vec3 computeEye() const;
};

} // namespace VPC
