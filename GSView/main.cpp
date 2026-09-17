#include "GSViewApp.h"

#include <cstdio>
#include <charconv>
#include <string>
#include <string_view>

namespace {

bool hasExtension(std::string_view a, std::string_view ext)
{
    return a.size() > ext.size() && a.substr(a.size() - ext.size()) == ext;
}

bool parseFloat(std::string_view value, float& out)
{
    const auto result = std::from_chars(value.data(), value.data() + value.size(), out);
    return result.ec == std::errc{} && result.ptr == value.data() + value.size();
}

} // namespace

int main(int argc, char* argv[])
{
    std::string scenarioPath;
    std::string plyPath;
    std::string renderMode;
    bool        noExitOnComplete    = false;
    bool        exitAfterScreenshot = false;
    bool        hideUI              = false;
    bool        hasScreenshot       = false;
    bool        hasCamTheta = false, hasCamPhi = false, hasCamDistance = false;
    float       camTheta = 0.f, camPhi = 0.f, camDistance = 0.f;
    int width = 1280, height = 720;

    for (int i = 1; i < argc; ++i) {
        std::string_view a = argv[i];
        if (a == "--run-scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if ((a == "--width" || a == "--height") && i + 1 < argc) {
            const std::string_view value = argv[++i];
            int parsed = 0;
            const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
                || parsed < 64 || parsed > 4096) {
                std::fprintf(stderr, "Invalid window extent (expected 64..4096)\n");
                return 1;
            }
            (a == "--width" ? width : height) = parsed;
        } else if (a == "--no-exit-on-complete") {
            noExitOnComplete = true;
        } else if (a == "--exit-after-screenshot") {
            exitAfterScreenshot = true;
        } else if (a == "--hide-ui") {
            hideUI = true;
        } else if (a == "--render-mode" && i + 1 < argc) {
            renderMode = argv[++i];
        } else if ((a == "--camera-theta" || a == "--camera-phi" || a == "--camera-distance")
                   && i + 1 < argc) {
            float parsed = 0.f;
            if (!parseFloat(argv[++i], parsed)) {
                std::fprintf(stderr, "Invalid value for %.*s\n", static_cast<int>(a.size()), a.data());
                return 1;
            }
            if (a == "--camera-theta")         { camTheta = parsed;    hasCamTheta = true; }
            else if (a == "--camera-phi")      { camPhi = parsed;      hasCamPhi = true; }
            else                                { camDistance = parsed; hasCamDistance = true; }
        } else if ((a == "--screenshot" || a == "--screenshot-frame") && i + 1 < argc) {
            if (a == "--screenshot") hasScreenshot = true;
            ++i; // consumed by VkAppBase::run()
        } else if (a.rfind("--screenshot=", 0) == 0) {
            hasScreenshot = true;
        } else if (a.rfind("--screenshot-frame=", 0) == 0) {
            // no extra arg
        } else if (a.rfind("--", 0) != 0
                   && (hasExtension(a, ".ply") || hasExtension(a, ".splat"))) {
            plyPath = std::string(a);
        }
    }

    if ((hasCamTheta || hasCamPhi || hasCamDistance)
        && !(hasCamTheta && hasCamPhi && hasCamDistance)) {
        std::fprintf(stderr,
                      "--camera-theta/--camera-phi/--camera-distance must be given together\n");
        return 1;
    }
    if (exitAfterScreenshot && !hasScreenshot) {
        std::fprintf(stderr,
                      "[GSView] --exit-after-screenshot has no effect without --screenshot\n");
    }

    GSView::GSViewApp app(width, height, "GS View");

    if (!plyPath.empty())
        app.setInitialPLY(plyPath);
    if (!renderMode.empty())
        app.setInitialRenderMode(renderMode);
    if (hasCamTheta)
        app.setInitialCamera(camTheta, camPhi, camDistance);
    app.setExitAfterScreenshot(exitAfterScreenshot);
    // --exit-after-screenshot is a one-shot renderer workflow: hide UI by default
    // unless the caller passes --hide-ui explicitly (this only ever adds hiding,
    // never disables an explicit --hide-ui).
    app.setHideUI(hideUI || exitAfterScreenshot);

    if (!scenarioPath.empty()) {
        if (!app.loadScenario(scenarioPath)) {
            std::fprintf(stderr, "[Scenario] Failed to load: %s\n", scenarioPath.c_str());
            return 1;
        }
        app.setExitOnScenarioComplete(!noExitOnComplete);
    }

    app.run(argc, argv);
    return app.getExitCode();
}
