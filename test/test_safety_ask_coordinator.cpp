#include <string>
#include <unordered_set>

#include "gtest/gtest.h"

#include "sura_bt/safety_ask_coordinator.hpp"

namespace
{
std::string treeXml(const std::string & prefix)
{
  return "<root><BehaviorTree ID=\"AutonomousBranch\"><Sequence>" + prefix +
    "<MissionCheckpoint key=\"done\"><GoToPoseAction name=\"reached\"/>"
    "</MissionCheckpoint></Sequence></BehaviorTree></root>";
}
}  // namespace

TEST(SafetyAskCoordinator, ExposesCompletedActionForAgentContext)
{
  const auto actions = sura_bt::SafetyAskCoordinator::checkpointActions(treeXml(""));
  ASSERT_EQ(actions.size(), 1u);
  EXPECT_NE(actions.at("done").second.find("GoToPoseAction"), std::string::npos);
  EXPECT_NE(actions.at("done").first.find("Sequence"), std::string::npos);
}

TEST(SafetyAskCoordinator, RejectsMovingOrRemovingCompletedAction)
{
  const auto original = sura_bt::SafetyAskCoordinator::checkpointActions(treeXml(""));
  const std::unordered_set<std::string> completed{"done"};
  EXPECT_NO_THROW(sura_bt::SafetyAskCoordinator::validateCompletedActions(
    original, original, completed));

  const auto moved = sura_bt::SafetyAskCoordinator::checkpointActions(
    treeXml("<AlwaysSuccess/>"));
  EXPECT_THROW(sura_bt::SafetyAskCoordinator::validateCompletedActions(
    original, moved, completed), std::runtime_error);

  const auto missing = sura_bt::SafetyAskCoordinator::checkpointActions(
    "<root><BehaviorTree ID=\"AutonomousBranch\"><Sequence/></BehaviorTree></root>");
  EXPECT_THROW(sura_bt::SafetyAskCoordinator::validateCompletedActions(
    original, missing, completed), std::runtime_error);

  const auto changed = sura_bt::SafetyAskCoordinator::checkpointActions(
    "<root><BehaviorTree ID=\"AutonomousBranch\"><Sequence>"
    "<MissionCheckpoint key=\"done\"><GoToPoseAction name=\"different\"/>"
    "</MissionCheckpoint></Sequence></BehaviorTree></root>");
  EXPECT_THROW(sura_bt::SafetyAskCoordinator::validateCompletedActions(
    original, changed, completed), std::runtime_error);
}
