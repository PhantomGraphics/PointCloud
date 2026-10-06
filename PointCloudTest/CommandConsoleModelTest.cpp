#include "pch.h"

// docs/todo/PLAN_viewer_command_ui.md Phase 1: the ImGui-free state of the shared
// Command window (response ownership between console and ScenarioRunner,
// history, completion, help).
#include "CGLib/VkAppBase/ScenarioRunner/CommandConsoleModel.h"

namespace {

const std::vector<CommandInfo> kCatalog = {
    {"GetStatus", "", "alive?"},
    {"GetSplatCount", "", ""},
    {"SetRenderMode", "SortBased|GaussianPoint", "switch mode"},
    {"SetGpSpp", "int", ""},
    {"SetGpSeed", "uint", ""},
};

using Model = CommandConsoleModel;

} // namespace

TEST(CommandConsoleModel, DispatchesCommandTextUnchangedExceptOuterWhitespace)
{
    Model m;
    std::string cmd;
    ASSERT_EQ(m.submit("  LoadPLY:C:\\My Data\\a b.ply \t", kCatalog, cmd), Model::SubmitResult::Dispatch);
    EXPECT_EQ(cmd, "LoadPLY:C:\\My Data\\a b.ply");  // inner spaces / colons / backslashes survive
    EXPECT_EQ(m.pending(), 1u);
}

TEST(CommandConsoleModel, BlankLineIsIgnored)
{
    Model m;
    std::string cmd;
    EXPECT_EQ(m.submit("   ", kCatalog, cmd), Model::SubmitResult::Ignored);
    EXPECT_EQ(m.pending(), 0u);
    EXPECT_TRUE(m.lines().empty());
}

TEST(CommandConsoleModel, ConsumesOnlyItsOwnResponsesAndLeavesTheRestForTheScenario)
{
    Model m;
    std::string cmd;
    m.submit("GetStatus", kCatalog, cmd);
    m.submit("GetSplatCount", kCatalog, cmd);
    ASSERT_EQ(m.pending(), 2u);

    // Dispatcher answered both manual commands and (later in the same frame
    // batch) the scenario's first command.
    std::vector<std::string> responses = {"OK", "Val:5", "Val:scenario"};
    m.consumeResponses(responses);

    ASSERT_EQ(responses.size(), 1u);
    EXPECT_EQ(responses[0], "Val:scenario");
    EXPECT_EQ(m.pending(), 0u);
    ASSERT_EQ(m.lines().size(), 4u);  // 2 echoes + 2 results
    EXPECT_EQ(m.lines()[2].text, "OK");
    EXPECT_EQ(m.lines()[3].text, "Val:5");
}

TEST(CommandConsoleModel, NothingPendingMeansEveryResponseGoesToTheScenario)
{
    Model m;
    std::vector<std::string> responses = {"OK", "Val:1"};
    m.consumeResponses(responses);
    EXPECT_EQ(responses.size(), 2u);
    EXPECT_TRUE(m.lines().empty());
}

TEST(CommandConsoleModel, ResponseArrivingLaterKeepsPendingCount)
{
    Model m;
    std::string cmd;
    m.submit("GetStatus", kCatalog, cmd);
    m.submit("GetSplatCount", kCatalog, cmd);

    std::vector<std::string> first = {"OK"};
    m.consumeResponses(first);
    EXPECT_TRUE(first.empty());
    EXPECT_EQ(m.pending(), 1u);

    std::vector<std::string> second = {"Val:5"};
    m.consumeResponses(second);
    EXPECT_TRUE(second.empty());
    EXPECT_EQ(m.pending(), 0u);
}

TEST(CommandConsoleModel, ErrorResponsesAreClassified)
{
    Model m;
    std::string cmd;
    m.submit("Nope", kCatalog, cmd);
    std::vector<std::string> responses = {"Error:unknown command Nope"};
    m.consumeResponses(responses);
    EXPECT_EQ(m.lines().back().kind, Model::Line::Kind::Error);
}

TEST(CommandConsoleModel, HelpAndClearAreLocalAndNeverDispatched)
{
    Model m;
    std::string cmd;
    EXPECT_EQ(m.submit("help", kCatalog, cmd), Model::SubmitResult::Local);
    EXPECT_EQ(m.pending(), 0u);
    EXPECT_GT(m.lines().size(), 2u);

    const size_t before = m.lines().size();
    EXPECT_EQ(m.submit("help SetRenderMode", kCatalog, cmd), Model::SubmitResult::Local);
    ASSERT_GE(m.lines().size(), before + 3);
    EXPECT_EQ(m.lines()[before + 1].text, "SetRenderMode:SortBased|GaussianPoint");

    EXPECT_EQ(m.submit("clear", kCatalog, cmd), Model::SubmitResult::Local);
    EXPECT_TRUE(m.lines().empty());
}

TEST(CommandConsoleModel, HelpForUnknownCommandIsAnErrorLine)
{
    Model m;
    std::string cmd;
    m.submit("help Bogus", kCatalog, cmd);
    EXPECT_EQ(m.lines().back().kind, Model::Line::Kind::Error);
}

TEST(CommandConsoleModel, HelpWithEmptyCatalogDoesNotCrash)
{
    Model m;
    std::string cmd;
    EXPECT_EQ(m.submit("help", {}, cmd), Model::SubmitResult::Local);
}

TEST(CommandConsoleModel, HistoryWalksBackAndForward)
{
    Model m;
    std::string cmd;
    m.submit("A", kCatalog, cmd);
    m.submit("B", kCatalog, cmd);
    m.submit("B", kCatalog, cmd);  // duplicate of the last entry is not stored twice
    EXPECT_EQ(m.historyPrev(), "B");
    EXPECT_EQ(m.historyPrev(), "A");
    EXPECT_EQ(m.historyPrev(), "A");  // stays at the oldest
    EXPECT_EQ(m.historyNext(), "B");
    EXPECT_EQ(m.historyNext(), "");   // past the newest -> empty input
}

TEST(CommandConsoleModel, CompletionOnlyOffersCatalogNames)
{
    EXPECT_EQ(Model::complete("SetGp", kCatalog), (std::vector<std::string>{"SetGpSeed", "SetGpSpp"}));
    EXPECT_TRUE(Model::complete("Frobnicate", kCatalog).empty());
    EXPECT_EQ(Model::complete("", kCatalog).size(), kCatalog.size());
}

TEST(CommandConsoleModel, NoCompletionOnceTheArgumentStarted)
{
    EXPECT_TRUE(Model::complete("SetGpSpp:", kCatalog).empty());
}

TEST(CommandConsoleModel, CommonPrefixOfCandidates)
{
    EXPECT_EQ(Model::commonPrefix({"SetGpSeed", "SetGpSpp"}), "SetGpS");
    EXPECT_EQ(Model::commonPrefix({"GetStatus"}), "GetStatus");
    EXPECT_EQ(Model::commonPrefix({}), "");
}
