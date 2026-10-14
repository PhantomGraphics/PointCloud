#include "pch.h"

#include "CGLib/VkAppBase/ScenarioRunner/CommandConsoleModel.h"
#include "CGLib/VkAppBase/ScenarioRunner/IScenarioDispatcher.h"
#include <type_traits>

namespace {
class ConsoleTestDispatcher : public IScenarioDispatcher {
public:
    void dispatch(const std::string& command) override {
        responses.push_back("OK:" + command);
    }
    std::vector<std::string> collectResponses() override {
        auto out = std::move(responses);
        responses.clear();
        return out;
    }
    std::vector<CommandInfo> commandCatalog() const override {
        return {{"GetStatus", "", "alive?"}};
    }
private:
    std::vector<std::string> responses;
};
}

TEST(CommandConsoleIntegration, LegacyHeadersUseEditorTypesAndDispatcherCatalog) {
    static_assert(std::is_same_v<CommandInfo, Phantom::Editor::CommandInfo>);
    static_assert(std::is_same_v<CommandConsoleModel, Phantom::Editor::CommandConsoleModel>);
    ConsoleTestDispatcher dispatcher;
    CommandConsoleModel model;
    std::string command;
    for (bool scenario : {false, true}) {
        ASSERT_EQ(model.submit("GetStatus", dispatcher.commandCatalog(), command, scenario),
            CommandConsoleModel::SubmitResult::Dispatch);
        dispatcher.dispatch(command);
    }
    auto responses = dispatcher.collectResponses();
    model.consumeResponses(responses);
    EXPECT_EQ(responses, (std::vector<std::string>{"OK:GetStatus"}));
    EXPECT_EQ(model.pending(), 0u);
    EXPECT_EQ(CommandConsoleModel::complete("Get", dispatcher.commandCatalog()),
        (std::vector<std::string>{"GetStatus"}));
}
