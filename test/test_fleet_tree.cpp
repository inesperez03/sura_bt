#include <string>
#include <vector>
#include <memory>
#include <unordered_set>

#include "gtest/gtest.h"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sura_bt/fleet_tree.hpp"
#include "sura_bt/operation_mode_nodes.hpp"
#include "tinyxml2.h"

namespace
{

class RunningSafety : public BT::StatefulActionNode
{
public:
  RunningSafety(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {}
};

class TrackedGoal : public BT::StatefulActionNode
{
public:
  TrackedGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return tickGoal();}
  BT::NodeStatus onRunning() override {return tickGoal();}
  void onHalted() override
  {
    config().blackboard->set("goal_halts", config().blackboard->get<int>("goal_halts") + 1);
  }

private:
  BT::NodeStatus tickGoal()
  {
    config().blackboard->set("goal_ticks", config().blackboard->get<int>("goal_ticks") + 1);
    return BT::NodeStatus::RUNNING;
  }
};

}  // namespace

TEST(FleetTree, ParsesExplicitRobotList)
{
  EXPECT_EQ(
    sura_bt::parseRobotNamespaces("/bluerov/, blueboat, cirtesub"),
    (std::vector<std::string>{"bluerov", "blueboat", "cirtesub"}));
  EXPECT_THROW(sura_bt::parseRobotNamespaces("bluerov,bluerov"), std::invalid_argument);
  EXPECT_THROW(sura_bt::parseRobotNamespaces("bluerov,"), std::invalid_argument);
  EXPECT_THROW(sura_bt::parseRobotNamespaces("blue/rov"), std::invalid_argument);
}

TEST(FleetTree, DiscoversRobotsFromSourceIdentity)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  auto add_status = [&](const std::string & display, const std::string & source, unsigned char level) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = display;
      status.level = level;
      diagnostic_msgs::msg::KeyValue identity;
      identity.key = "source_name";
      identity.value = source;
      status.values.push_back(identity);
      array.status.push_back(status);
    };
  add_status("/ROBOTS/BLUEROV/Sensors/IMU", "/bluerov/Sensors/IMU", 0);
  add_status("/ROBOTS/BLUEBOAT/Sensors/GPS", "/blueboat/Sensors/GPS", 1);
  add_status("/ROBOTS/OLD/Sensors/GPS", "/old/Sensors/GPS", 3);

  EXPECT_EQ(
    sura_bt::robotsFromDiagnostics(array),
    (std::vector<std::string>{"blueboat", "bluerov"}));
}

TEST(FleetTree, IsolatesEachRobotSafetyBlackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::ReactiveParallel>("ReactiveParallel");
  factory.registerSimpleAction(
    "PublishSafety",
    [](BT::TreeNode & node) {
      const auto robot = node.config().blackboard->get<std::string>("robot_namespace");
      node.config().blackboard->set("mission_control", robot == "blueboat" ? "pause" : "run");
      return BT::NodeStatus::SUCCESS;
    });
  factory.registerBehaviorTreeFromText(
    "<root><BehaviorTree ID=\"SafetyBranch_blueboat\"><PublishSafety/>"
    "</BehaviorTree></root>");
  factory.registerBehaviorTreeFromText(
    "<root><BehaviorTree ID=\"SafetyBranch_bluerov\"><PublishSafety/>"
    "</BehaviorTree></root>");

  const std::string template_file = std::string(SURA_BT_SOURCE_DIR) + "/trees/fleet_safety.xml";
  factory.registerBehaviorTreeFromText(
    sura_bt::expandedFleetSafetyXml(template_file, {"blueboat", "bluerov"}));
  auto blackboard = BT::Blackboard::create();
  blackboard->set("robot_namespaces", std::string("blueboat,bluerov"));
  blackboard->set("robot_namespace_blueboat", std::string("blueboat"));
  blackboard->set("robot_namespace_bluerov", std::string("bluerov"));
  auto tree = factory.createTree("SafetyBranch", blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("mission_control_blueboat"), "pause");
  EXPECT_EQ(blackboard->get<std::string>("mission_control_bluerov"), "run");
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "pause");
}

TEST(FleetTree, StandardParallelDoesNotRestartCompletedMissionAction)
{
  BT::BehaviorTreeFactory factory;
  int completed_ticks = 0;
  factory.registerSimpleAction(
    "CompletedGoal",
    [&](BT::TreeNode &) {
      ++completed_ticks;
      return BT::NodeStatus::SUCCESS;
    });
  factory.registerNodeType<RunningSafety>("RunningGoal");

  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Mission\"><BehaviorTree ID=\"Mission\">"
    "<Parallel success_threshold=\"2\" failure_threshold=\"1\">"
    "<CompletedGoal/><RunningGoal/></Parallel>"
    "</BehaviorTree></root>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(completed_ticks, 1);
  tree.haltTree();
}

TEST(FleetTree, ContinuesCheckingSuccessfulRobotDuringAnotherRecovery)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::ReactiveParallel>("ReactiveParallel");
  factory.registerNodeType<RunningSafety>("RunningSafety");
  int checks = 0;
  factory.registerSimpleAction(
    "CheckSafety",
    [&](BT::TreeNode &) {
      ++checks;
      return BT::NodeStatus::SUCCESS;
    });

  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"CheckAll\"><BehaviorTree ID=\"CheckAll\">"
    "<ReactiveParallel><CheckSafety/><RunningSafety/></ReactiveParallel>"
    "</BehaviorTree></root>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(checks, 2);
  tree.haltTree();
}

TEST(FleetTree, RecoverableSafetyErrorDoesNotStopAutonomousMission)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::MissionControl>("MissionControl");
  factory.registerNodeType<TrackedGoal>("TrackedGoal");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("mission_control", std::string("run"));
  blackboard->set("goal_ticks", 0);
  blackboard->set("goal_halts", 0);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Mission\"><BehaviorTree ID=\"Mission\">"
    "<MissionControl><TrackedGoal/></MissionControl>"
    "</BehaviorTree></root>", blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  blackboard->set("mission_control", std::string("pause"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 2);
  EXPECT_EQ(blackboard->get<int>("goal_halts"), 0);

  blackboard->set("mission_control", std::string("abort"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 2);
  EXPECT_EQ(blackboard->get<int>("goal_halts"), 1);
}

TEST(FleetTree, SafetyAskHaltsGoalUntilResolved)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::MissionControl>("MissionControl");
  factory.registerNodeType<TrackedGoal>("TrackedGoal");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("mission_control", std::string("run"));
  blackboard->set("goal_ticks", 0);
  blackboard->set("goal_halts", 0);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Mission\"><BehaviorTree ID=\"Mission\">"
    "<MissionControl><TrackedGoal/></MissionControl>"
    "</BehaviorTree></root>", blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  blackboard->set("mission_control", std::string("ask"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 1);
  EXPECT_EQ(blackboard->get<int>("goal_halts"), 1);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 1);
  blackboard->set("mission_control", std::string("run"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 2);
}

TEST(FleetTree, MissionCheckpointPreservesCompletedParallelSiblingAcrossReload)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::MissionCheckpoint>("MissionCheckpoint");
  factory.registerNodeType<TrackedGoal>("TrackedGoal");
  int completed_ticks = 0;
  factory.registerSimpleAction("CompletedGoal", [&](BT::TreeNode &) {
    ++completed_ticks;
    return BT::NodeStatus::SUCCESS;
  });
  auto blackboard = BT::Blackboard::create();
  blackboard->set("goal_ticks", 0);
  blackboard->set("goal_halts", 0);
  blackboard->set("mission_completed_actions",
    std::make_shared<std::unordered_set<std::string>>());
  const std::string xml =
    "<root main_tree_to_execute=\"Mission\"><BehaviorTree ID=\"Mission\">"
    "<Parallel success_threshold=\"2\" failure_threshold=\"1\">"
    "<MissionCheckpoint key=\"done\"><CompletedGoal/></MissionCheckpoint>"
    "<MissionCheckpoint key=\"active\"><TrackedGoal/></MissionCheckpoint>"
    "</Parallel></BehaviorTree></root>";
  auto first = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(first.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(completed_ticks, 1);
  first.haltTree();
  auto reloaded = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(reloaded.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(completed_ticks, 1);
  EXPECT_EQ(blackboard->get<int>("goal_ticks"), 2);
}

TEST(FleetTree, AutonomousActionsReceiveCheckpoints)
{
  const auto file = std::string(SURA_BT_SOURCE_DIR) + "/test/fixtures/autonomous_checkpoint.xml";
  const auto xml = sura_bt::checkpointedAutonomousTreeXml(file);
  tinyxml2::XMLDocument document;
  ASSERT_EQ(document.Parse(xml.c_str()), tinyxml2::XML_SUCCESS);
  auto * root = document.FirstChildElement("root");
  auto * tree = root ? root->FirstChildElement("BehaviorTree") : nullptr;
  auto * sequence = tree ? tree->FirstChildElement("Sequence") : nullptr;
  ASSERT_NE(sequence, nullptr);
  EXPECT_NE(xml.find("<MissionCheckpoint"), std::string::npos);
  EXPECT_NE(xml.find("<SendVelocity"), std::string::npos);
  EXPECT_NE(xml.find("<SurfaceAction"), std::string::npos);
}
