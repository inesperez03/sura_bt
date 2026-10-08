#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sura_bt/avoid_obstacle.hpp"
#include "sura_safety/diagnostics_monitor.hpp"

namespace
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
using Array = diagnostic_msgs::msg::DiagnosticArray;
using Velocity = sura_msgs::msg::SuraVelocityCommand;
using Mode = sura_actions::srv::SetControlMode;

class AvoidObstacleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node_ = std::make_shared<rclcpp::Node>("avoid_obstacle_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    monitor_ = std::make_shared<sura_safety::DiagnosticsMonitor>(
      node_, "/test_avoid_diagnostics", "robot_a");
    diagnostics_ = node_->create_publisher<Array>("/test_avoid_diagnostics", 10);
    velocity_sub_ = node_->create_subscription<Velocity>(
      "/robot_a/controller/arbitrator/velocity", 10,
      [this](Velocity::ConstSharedPtr command) {commands_.push_back(*command);});
    mode_service_ = node_->create_service<Mode>(
      "/robot_a/control_manager/set_mode",
      [this](const std::shared_ptr<Mode::Request> request,
        std::shared_ptr<Mode::Response> response) {
        requested_modes_.push_back(request->mode);
        response->success = true;
      });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("ros_node", node_);
    blackboard_->set("diagnostics_monitor", monitor_);
    blackboard_->set("robot_family_robot_a", std::string("underwater"));
    blackboard_->set("robot_available_axes_robot_a", std::vector<std::string>{"surge", "sway"});
    factory_.registerNodeType<sura_bt::AvoidObstacle>("AvoidObstacle");
  }

  void TearDown() override
  {
    executor_->remove_node(node_);
    executor_.reset();
    mode_service_.reset();
    velocity_sub_.reset();
    diagnostics_.reset();
    monitor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  BT::Tree tree(const std::string & attributes = "")
  {
    return factory_.createTreeFromText(
      "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
      "<AvoidObstacle robot_namespace=\"robot_a\" " + attributes + "/>"
      "</BehaviorTree></root>", blackboard_);
  }

  void diagnostic(uint8_t level, const std::string & message)
  {
    Array array;
    Status status;
    status.name = "/display/front";
    status.level = level;
    status.message = message;
    diagnostic_msgs::msg::KeyValue source;
    source.key = "source_name";
    source.value = "/robot_a/Navigation/FrontObstacle";
    status.values.push_back(source);
    array.status.push_back(status);
    diagnostics_->publish(array);
    executor_->spin_some();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<sura_safety::DiagnosticsMonitor> monitor_;
  rclcpp::Publisher<Array>::SharedPtr diagnostics_;
  rclcpp::Subscription<Velocity>::SharedPtr velocity_sub_;
  rclcpp::Service<Mode>::SharedPtr mode_service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  std::vector<Velocity> commands_;
  std::vector<uint8_t> requested_modes_;
};

TEST_F(AvoidObstacleTest, StartsOnlyForARealFrontObstacleAndAvailableSway)
{
  auto missing = tree();
  EXPECT_EQ(missing.tickRoot(), BT::NodeStatus::FAILURE);

  diagnostic(Status::OK, "Front obstacle distance healthy");
  auto clear = tree();
  EXPECT_EQ(clear.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(commands_.empty());

  diagnostic(Status::WARN, "Front obstacle close");
  auto warning = tree();
  EXPECT_EQ(warning.tickRoot(), BT::NodeStatus::FAILURE);

  diagnostic(Status::ERROR, "Depth image encoding is not supported");
  auto invalid = tree();
  EXPECT_EQ(invalid.tickRoot(), BT::NodeStatus::FAILURE);

  diagnostic(Status::STALE, "Front obstacle distance is unknown");
  auto stale = tree();
  EXPECT_EQ(stale.tickRoot(), BT::NodeStatus::FAILURE);

  diagnostic(Status::ERROR, "Front obstacle critically close");
  blackboard_->set("robot_available_axes_robot_a", std::vector<std::string>{"surge"});
  auto no_sway = tree();
  EXPECT_EQ(no_sway.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(AvoidObstacleTest, SlidesUntilClearAndPublishesZeroOnHalt)
{
  diagnostic(Status::ERROR, "Front obstacle critically close");
  auto right = tree("side=\"right\"");
  EXPECT_EQ(right.tickRoot(), BT::NodeStatus::RUNNING);
  for (int attempt = 0; attempt < 100 && commands_.empty(); ++attempt) {
    executor_->spin_some();
    EXPECT_EQ(right.tickRoot(), BT::NodeStatus::RUNNING);
    executor_->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.2);
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().velocity.angular.z, 0.0);
  ASSERT_FALSE(requested_modes_.empty());
  EXPECT_EQ(requested_modes_.back(), Mode::Request::BODY_VELOCITY);


  diagnostic(Status::WARN, "Front obstacle close");
  EXPECT_EQ(right.tickRoot(), BT::NodeStatus::RUNNING);
  diagnostic(Status::OK, "Front obstacle distance healthy");
  EXPECT_EQ(right.tickRoot(), BT::NodeStatus::SUCCESS);
  executor_->spin_some();
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.0);

  diagnostic(Status::ERROR, "Front obstacle critically close");
  auto left = tree("side=\"left\"");
  EXPECT_EQ(left.tickRoot(), BT::NodeStatus::RUNNING);
  commands_.clear();
  for (int attempt = 0; attempt < 100 && commands_.empty(); ++attempt) {
    executor_->spin_some();
    EXPECT_EQ(left.tickRoot(), BT::NodeStatus::RUNNING);
    executor_->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, -0.2);
  left.haltTree();
  executor_->spin_some();
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.0);
}

TEST_F(AvoidObstacleTest, ContinuesUntilObstacleClearsRegardlessOfElapsedTime)
{
  diagnostic(Status::ERROR, "Front obstacle critically close");
  auto avoiding = tree();
  EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::RUNNING);
  for (int attempt = 0; attempt < 40 && commands_.empty(); ++attempt) {
    executor_->spin_some();
    EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::RUNNING);
    executor_->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.2);
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::RUNNING);
  diagnostic(Status::OK, "Front obstacle distance healthy");
  EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::SUCCESS);
  executor_->spin_some();
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.0);
}

TEST_F(AvoidObstacleTest, StopsWhenTheDiagnosticBecomesInvalid)
{
  diagnostic(Status::ERROR, "Front obstacle critically close");
  auto avoiding = tree();
  EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::RUNNING);
  for (int attempt = 0; attempt < 100 && commands_.empty(); ++attempt) {
    executor_->spin_some();
    EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::RUNNING);
    executor_->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(commands_.empty());
  diagnostic(Status::ERROR, "Depth image encoding is not supported");
  EXPECT_EQ(avoiding.tickRoot(), BT::NodeStatus::FAILURE);
  executor_->spin_some();
  EXPECT_DOUBLE_EQ(commands_.back().velocity.linear.y, 0.0);
}
}  // namespace
