#include "PointCloudApp.h"

#include "PointCloudFileLoader.h"
#include "GSPointPresenter.h"

#include "../PointCloud/GSPointCloud.h"

#include <filesystem>
#include <fstream>
#include <string>

namespace VPC {

namespace {

bool isGaussianSplatPly(const std::filesystem::path& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) return false;

    bool hasFdc = false, hasOpacity = false, hasScale = false, hasRot = false;
    std::string line;
    int guard = 0;
    while (std::getline(ifs, line) && guard++ < 512) {
        if (line.find("property") != std::string::npos) {
            if (line.find("f_dc_0")  != std::string::npos) hasFdc     = true;
            if (line.find("opacity") != std::string::npos) hasOpacity = true;
            if (line.find("scale_0") != std::string::npos) hasScale   = true;
            if (line.find("rot_0")   != std::string::npos) hasRot     = true;
        }
        if (line == "end_header") break;
    }
    return hasFdc && hasOpacity && hasScale && hasRot;
}

} // namespace

// ============================================================
//  Construction
// ============================================================

PointCloudApp::PointCloudApp(int width, int height, const std::string& title)
    : VkAppBase(width, height, title)
{
    renderer_.setWorld(&world_);

    sceneListPanel_.init(
        &world_,
        renderer_.getActiveSceneIdPtr(),
        [this]() { syncRenderer(); },
        [this]() { renderer_.notifySceneSelectionChanged(); });

    processPanel_.init(
        &world_,
        renderer_.getActiveSceneIdPtr(),
        [this]() { syncRenderer(); });

    importExportPanel_.init(
        &renderer_,
        [this](const std::string& p, std::string& e) { return loadPointCloudFromFile(p, e); },
        [this](const std::string& p, std::string& e) { return savePointCloudToFile(p, e); });

    menu_.init(&controlHost_, &processPanel_);

    dispatcher_.setWorld(&world_);
    dispatcher_.setOnWorldChanged([this](int id) {
        renderer_.setActiveSceneId(id);
        syncRenderer();
    });
    scenarioBrowser_.setHost(this);
    scenarioBrowser_.setDefaultFolder("scenarios");

    registerControlPages();
    controlHost_.setStatusDrawer([this]() { drawStatusArea(); });
    controlHost_.setProcessAccessors(
        [this]() { return static_cast<int>(processPanel_.getProcess()); },
        [this](int id) {
            processPanel_.setProcess(
                (id >= 0 && id < kProcessCount) ? static_cast<ProcessId>(id)
                                                : ProcessId::None);
        });
    controlHost_.setLayoutFile("pointcloudview_control_layout.ini");

    add(&renderer_);
    add(&controlHost_);
}

void PointCloudApp::registerControlPages()
{
    scenesEmbed_.setFunction([this]() { sceneListPanel_.onImGui(); });
    renderingEmbed_.setFunction([this]() { renderer_.drawImGuiControls(); });
    // ScenarioBrowserPanel lives in CGLib and cannot derive from the
    // PointCloudView-local IEmbeddedPanel, so bridge it through a callable.
    scenarioBrowserEmbed_.setFunction([this]() { scenarioBrowser_.drawEmbedded(); });

    controlHost_.registerPage(ControlPage::Scenes,          &scenesEmbed_);
    controlHost_.registerPage(ControlPage::Rendering,       &renderingEmbed_);
    controlHost_.registerPage(ControlPage::Processing,      &processPanel_);
    controlHost_.registerPage(ControlPage::ImportExport,    &importExportPanel_);
    controlHost_.registerPage(ControlPage::ScenarioBrowser, &scenarioBrowserEmbed_);
}

// ============================================================
//  Public API
// ============================================================

bool PointCloudApp::loadPointCloudFromFile(const std::string& path,
                                            std::string& errorMessage)
{
    errorMessage.clear();
    const auto ext = std::filesystem::path(path).extension().string();
    if (ext == ".ply" && isGaussianSplatPly(path)) {
        Phantom::PointCloud::GSPointCloud gsCloud;
        if (!gsCloud.readFromFile(path)) {
            errorMessage = "Failed to read GS-PLY";
            return false;
        }

        auto splats = GSPointPresenter::build(gsCloud);
        const auto name = std::filesystem::path(path).filename().string();
        auto* scene = world_.addScene(name);
        scene->setGSSplats(splats);
        for (const auto& s : splats)
            scene->add(glm::vec3(s.centerSize), glm::vec3(s.color));

        renderer_.setActiveSceneId(scene->getId());
        renderer_.setRenderMode(
            renderer_.isGaussianSplattingAvailable()
            ? PointCloudRenderer::RenderMode::GaussianSplatting
            : PointCloudRenderer::RenderMode::Point);
        syncRenderer();
        return true;
    }

    Phantom::PC::PointCloudColoredData data;
    if (!Phantom::PC::loadPointCloud(path, data, errorMessage))
        return false;
    const auto name = std::filesystem::path(path).filename().string();
    auto* scene = world_.addScene(name);
    for (size_t i = 0; i < data.size(); ++i)
        scene->add(data.positions[i], data.colors[i]);
    renderer_.setActiveSceneId(scene->getId());
    syncRenderer();
    return true;
}

bool PointCloudApp::savePointCloudToFile(const std::string& path,
                                          std::string& errorMessage) const
{
    errorMessage.clear();
    const auto& pts = renderer_.getCurrentPoints();
    Phantom::PC::PointCloudColoredData data;
    data.positions.reserve(pts.size());
    data.colors.reserve(pts.size());
    for (const auto& v : pts) {
        data.positions.push_back(v.pos);
        data.colors.push_back(v.color);
    }
    return Phantom::PC::savePointCloud(path, data, errorMessage);
}

bool PointCloudApp::loadScenario(const std::string& jsonPath) {
    // Scenario runs must not read or write the interactive Control layout.
    disableInteractiveLayoutPersistence();
    return runner_.load(jsonPath);
}

// ============================================================
//  VkAppBase hooks
// ============================================================

void PointCloudApp::onInit() {
    {
        static constexpr auto kSh = "shaders/";
        PointCloudRenderer::Shaders s;
        s.pointVert = ::VKG::loadSPVRepo(std::string(kSh) + "point.vert.spv");
        s.pointFrag = ::VKG::loadSPVRepo(std::string(kSh) + "point.frag.spv");
        s.gsVert    = ::VKG::loadSPVRepo(std::string(kSh) + "gs_splat.vert.spv");
        s.gsFrag    = ::VKG::loadSPVRepo(std::string(kSh) + "gs_splat.frag.spv");
        s.gsComp    = ::VKG::loadSPVRepo(std::string(kSh) + "gs_sort.comp.spv");
        s.lineVert  = ::VKG::loadSPVRepo(std::string(kSh) + "line.vert.spv");
        s.lineFrag  = ::VKG::loadSPVRepo(std::string(kSh) + "line.frag.spv");
        s.triVert   = ::VKG::loadSPVRepo(std::string(kSh) + "triangle.vert.spv");
        s.triFrag   = ::VKG::loadSPVRepo(std::string(kSh) + "triangle.frag.spv");
        renderer_.setShaders(std::move(s));
    }
    ::VKG::VkAppBase::onInit();
    renderer_.setExtent(getExtent());
    setupCallbacks();
}

void PointCloudApp::onSwapChainCreated() {
    renderer_.setExtent(getExtent());
}

void PointCloudApp::onUpdate(uint32_t frameIndex) {
    dispatcher_.processQueue();

    // Keep the Scenario Browser's GUI run-queue advancing every frame, even
    // when its page is not the one currently shown in the Control window.
    scenarioBrowser_.pumpQueue();

    if (runner_.isActive()) {
        auto responses = dispatcher_.collectResponses();
        if (runner_.tick(dispatcher_, responses)) {
            if (runner_.hasFailed()) {
                fprintf(stderr, "[Scenario] FAILED: %s\n", runner_.failMessage().c_str());
                exitCode_ = 1;
            } else {
                fprintf(stdout, "[Scenario] PASSED (%zu steps)\n", runner_.stepCount());
                exitCode_ = 0;
            }
            if (exitOnComplete_) getWindow().close();
        }
    } else {
    }

    ::VKG::VkAppBase::onUpdate(frameIndex);
}

void PointCloudApp::onCleanup() {
    ::VKG::VkAppBase::onCleanup();
}

void PointCloudApp::onImGui() {
    drawMenuBar();
    ::VKG::VkAppBase::onImGui();
}

// ============================================================
//  Private
// ============================================================

void PointCloudApp::drawMenuBar() {
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Import / Export...")) {
            controlHost_.setPage(ControlPage::ImportExport);
            controlHost_.setVisible(true);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Quit"))
            glfwSetWindowShouldClose(getWindow().get(), GLFW_TRUE);
        ImGui::EndMenu();
    }

    menu_.onImGuiMenuBar();

    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Control Window", nullptr, controlHost_.isVisible()))
            controlHost_.setVisible(!controlHost_.isVisible());
        if (ImGui::MenuItem("Reset Layout"))
            controlHost_.resetLayout();
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void PointCloudApp::drawStatusArea() {
    const int activeId = renderer_.getActiveSceneId();
    auto* scene = world_.findById(activeId);

    size_t totalPoints = 0;
    for (const auto& s : world_.getScenes()) totalPoints += s->getSize();

    const char* mode =
        (renderer_.getRenderMode() == PointCloudRenderer::RenderMode::GaussianSplatting)
            ? "Gaussian Splatting" : "Point";

    if (scene) {
        ImGui::TextWrapped("Scene: %s  (id %d, %zu pts)",
                           scene->getName().c_str(), activeId, scene->getSize());
    } else {
        ImGui::TextDisabled("Scene: (none selected)");
    }
    ImGui::Text("%zu scene(s), %zu pts total", world_.getScenes().size(), totalPoints);
    ImGui::Text("Draw: %s  (%u pts shown)", mode, renderer_.getPointCount());

    if (!importExportPanel_.lastStatus().empty())
        ImGui::TextWrapped("Last I/O: %s", importExportPanel_.lastStatus().c_str());

    if (runner_.isActive())
        ImGui::Text("Scenario: running (%zu steps)", runner_.stepCount());
    else if (runner_.hasFailed())
        ImGui::TextWrapped("Scenario FAILED: %s", runner_.failMessage().c_str());
}

void PointCloudApp::syncRenderer() {
    renderer_.notifyWorldChanged();
}

void PointCloudApp::setupCallbacks() {
    auto& win = getWindow();
    win.onMouseButton = [this](int button, int action, int) {
        if (button == 0) renderer_.handleMouseButton(action == 1);
    };
    win.onCursorPos = [this](double x, double y) {
        renderer_.handleMouseMove(x, y);
    };
    win.onScroll = [this](double, double dy) {
        renderer_.handleScroll(dy);
    };
}

} // namespace VPC
