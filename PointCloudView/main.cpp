#include "PointCloudApp.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <string_view>

int main(int argc, char* argv[]) {
    std::string scenarioPath;
    std::string filePath;
    bool        noExitOnComplete = false;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--run-scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (arg == "--no-exit-on-complete") {
            noExitOnComplete = true;
        } else if ((arg == "--screenshot" || arg == "--screenshot-frame") && i + 1 < argc) {
            ++i; // consumed by VkAppBase::run()
        } else if (arg.rfind("--screenshot-frame=", 0) == 0) {
            // no extra arg
        } else if (arg.rfind("--", 0) != 0 && filePath.empty()) {
            filePath = argv[i];
        }
    }

    VPC::PointCloudApp app(1280, 720, "Vulkan Point Cloud Viewer [experimental]");

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
