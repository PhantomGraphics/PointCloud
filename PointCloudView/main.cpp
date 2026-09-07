#include "PointCloudApp.h"

#include <charconv>
#include <cstdio>
#include <iostream>
#include <string>
#include <string_view>

int main(int argc, char* argv[]) {
    std::string scenarioPath;
    std::string filePath;
    bool        noExitOnComplete = false;
    bool        captureMode      = false;
    int         winW = 1280, winH = 720;
    int         startupPage = -1, startupProcess = -1;

    auto parseInt = [](std::string_view s, int fallback) {
        int v = fallback;
        std::from_chars(s.data(), s.data() + s.size(), v);
        return v;
    };

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--run-scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (arg == "--no-exit-on-complete") {
            noExitOnComplete = true;
        } else if (arg == "--size" && i + 1 < argc) {
            std::string_view s = argv[++i];
            const auto x = s.find('x');
            int w = 0, h = 0;
            if (x != std::string_view::npos &&
                std::from_chars(s.data(), s.data() + x, w).ec == std::errc() &&
                std::from_chars(s.data() + x + 1, s.data() + s.size(), h).ec == std::errc() &&
                w > 0 && h > 0) {
                winW = w; winH = h;
            }
        } else if (arg == "--page" && i + 1 < argc) {
            startupPage = parseInt(argv[++i], -1);
        } else if (arg == "--process" && i + 1 < argc) {
            startupProcess = parseInt(argv[++i], -1);
        } else if (arg == "--screenshot" && i + 1 < argc) {
            captureMode = true;
            ++i; // path consumed by VkAppBase::run()
        } else if (arg.rfind("--screenshot=", 0) == 0) {
            captureMode = true;
        } else if (arg == "--screenshot-frame" && i + 1 < argc) {
            ++i; // consumed by VkAppBase::run()
        } else if (arg.rfind("--screenshot-frame=", 0) == 0) {
            // no extra arg
        } else if (arg.rfind("--", 0) != 0 && filePath.empty()) {
            filePath = argv[i];
        }
    }

    VPC::PointCloudApp app(winW, winH, "Vulkan Point Cloud Viewer [experimental]");
    app.setCaptureMode(captureMode);
    app.setStartupSelection(startupPage, startupProcess);

    if (!filePath.empty()) {
        std::string error;
        if (!app.loadPointCloudFromFile(filePath, error)) {
            std::cerr << "Load failed: " << error << "\n";
            return 1;
        }
    }

    if (!scenarioPath.empty()) {
        if (!app.loadScenario(scenarioPath)) {
            fprintf(stderr, "[Scenario] Failed to load: %s\n", scenarioPath.c_str());
            return 1;
        }
        app.setExitOnScenarioComplete(!noExitOnComplete);
    }

    app.run(argc, argv);
    return app.getExitCode();
}
