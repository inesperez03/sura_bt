#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "sura_actions/action/go_to_pose.hpp"
#include "sura_actions/action/surface.hpp"

namespace sura_bt
{

class SurfaceAction : public BT::SyncActionNode
{
public:
  using Surface = sura_actions::action::Surface;
  using GoalHandleSurface = rclcpp_action::ClientGoalHandle<Surface>;

  SurfaceAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string actionName(const BT::NodeConfiguration & config) const;

  rclcpp_action::Client<Surface>::SharedPtr action_client_;
};

class GoToPoseAction : public BT::StatefulActionNode
{
public:
  using GoToPose = sura_actions::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;
  using WrappedResult = GoalHandleGoToPose::WrappedResult;
  using ChangeState = lifecycle_msgs::srv::ChangeState;
  using GetState = lifecycle_msgs::srv::GetState;

  GoToPoseAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string actionName(const BT::NodeConfiguration & config) const;
  std::string lifecycleNodeName(const BT::NodeConfiguration & config) const;
  bool prepareLifecycle(
    const rclcpp::Node::SharedPtr & ros_node,
    double timeout_sec);
  bool getLifecycleState(
    const rclcpp::Node::SharedPtr & ros_node,
    const std::string & lifecycle_node,
    uint8_t & state_id,
    double timeout_sec);
  bool changeLifecycleState(
    const rclcpp::Node::SharedPtr & ros_node,
    const std::string & lifecycle_node,
    uint8_t transition_id,
    double timeout_sec);
  void stopLifecycle(
    const rclcpp::Node::SharedPtr & ros_node,
    double timeout_sec);
  BT::NodeStatus handleResult(const WrappedResult & wrapped_result);
  void resetGoalState(bool active);

  rclcpp_action::Client<GoToPose>::SharedPtr action_client_;
  std::mutex goal_mutex_;
  GoalHandleGoToPose::SharedPtr goal_handle_;
  WrappedResult wrapped_result_;
  bool goal_active_{false};
  bool goal_response_received_{false};
  bool goal_rejected_{false};
  bool result_received_{false};
  bool cancel_requested_{false};
  uint64_t active_goal_id_{0};
  uint64_t next_goal_id_{0};
  double active_goal_timeout_{120.0};
  rclcpp::Time active_goal_start_time_;
};

}  // namespace sura_bt
