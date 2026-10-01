#include "gtest/gtest.h"
#include "sura_bt/command_nodes.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sura_safety/diagnostics_monitor.hpp"

TEST(AreaDiagnostics, ResolvesSourceUnderArbitraryDisplayRootAndRejectsStale)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("area_diagnostics_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    auto monitor = std::make_shared<sura_safety::DiagnosticsMonitor>(
      node, "/test_area_diagnostics", "robot_a");
    auto diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/test_area_diagnostics", 10);
    auto navigator = node->create_publisher<sura_msgs::msg::Navigator>("/robot_a/navigator/navigation", 10);
    auto blackboard = BT::Blackboard::create();
    blackboard->set("ros_node", node);
    blackboard->set("robot_namespace", std::string("robot_a"));
    blackboard->set("diagnostics_monitor", monitor);
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<sura_bt::ComputeAreaRecoveryForce>("ComputeAreaRecoveryForce");
    auto tree = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
        <ComputeAreaRecoveryForce namespace="{robot_namespace}" diagnostic_name="Navigation/AreaLimit" force="40"
          output_x="{x}" output_y="{y}"/>
      </BehaviorTree></root>)", blackboard);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto position = std::make_unique<sura_msgs::msg::Navigator>();
    position->position.orientation.w = 1.0;
    navigator->publish(std::move(position));
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/arbitrary/display/root/area";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
      {"source_name", "/robot_a/Navigation/AreaLimit"}, {"current_x", "10"},
      {"current_y", "0"}, {"center_x", "0"}, {"center_y", "0"}, {"shape", "circle"}})
    {
      diagnostic_msgs::msg::KeyValue value;
      value.key = pair.first;
      value.value = pair.second;
      status.values.push_back(value);
    }
    auto send = [&]() {
      auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
      msg->status.push_back(status);
      diagnostics->publish(std::move(msg));
      executor.spin_some();
    };
    send();
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(blackboard->get<double>("x"), -40.0);
    EXPECT_DOUBLE_EQ(blackboard->get<double>("y"), 0.0);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    send();
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  }
  rclcpp::shutdown();
}
