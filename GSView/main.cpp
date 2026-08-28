#include "GSViewApp.h"

#include <cstdio>
#include <string>
#include <string_view>

namespace {

bool hasExtension(std::string_view a, std::string_view ext)
{
    return a.size() > ext.size() && a.substr(a.size() - ext.size()) == ext;
}

} // namespace

int main(int argc, char* argv[])
{
    std::string scenarioPath;
    std::string plyPath;
    bool        noExitOnComplete = false;

    for (int i = 1; i < argc; ++i) {
        std::string_view a = argv[i];
        if (a == "--run-scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (a == "--no-exit-on-complete") {
            noExitOnComplete = true;
        } else if ((a == "--screenshot" || a == "--screenshot-frame") && i + 1 < argc) {
            ++i; // consumed by VkAppBase::run()
        } else if (a.rfind("--screenshot-frame=", 0) == 0) {
            // no extra arg
        } else if (a.rfind("--", 0) != 0
                   && (hasExtension(a, ".ply") || hasExtension(a, ".splat"))) {
            plyPath = std::string(a);
        }
    }

    GSView::GSViewApp app(1280, 720, "GS View");

    if (!plyPath.empty())
        app.setInitialPLY(plyPath);

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
