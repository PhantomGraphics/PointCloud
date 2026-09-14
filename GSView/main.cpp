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

} // namespace

int main(int argc, char* argv[])
{
    std::string scenarioPath;
    std::string plyPath;
    bool        noExitOnComplete = false;
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
        } else if ((a == "--screenshot" || a == "--screenshot-frame") && i + 1 < argc) {
            ++i; // consumed by VkAppBase::run()
        } else if (a.rfind("--screenshot-frame=", 0) == 0) {
            // no extra arg
        } else if (a.rfind("--", 0) != 0
                   && (hasExtension(a, ".ply") || hasExtension(a, ".splat"))) {
            plyPath = std::string(a);
        }
    }

    GSView::GSViewApp app(width, height, "GS View");

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
