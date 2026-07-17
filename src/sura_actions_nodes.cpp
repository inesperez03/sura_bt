#include "sura_bt/sura_actions_nodes.hpp"

#include <chrono>
#include <cmath>
#include <future>

namespace sura_bt
{
namespace
{

std::string stripSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/')
  {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/')
  {
    value.pop_back();
  }
  return value;
}

rclcpp::Node::SharedPtr getRosNode(const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

double positiveOrDefault(double value, double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

SurfaceAction::SurfaceAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  action_client_ =
    rclcpp_action::create_client<Surface>(
      getRosNode(config),
      actionName(config));
}

BT::PortsList SurfaceAction::providedPorts()
{
  return {
    BT::InputPort<double>("target_depth"),
    BT::InputPort<double>("depth_tolerance"),
    BT::InputPort<double>("timeout"),
    BT::InputPort<double>("surface_force_z"),
    BT::OutputPort<double>("final_depth"),
    BT::OutputPort<std::string>("message")
  };
}

BT::NodeStatus SurfaceAction::tick()
{
  const auto ros_node = getRosNode(config());

  auto target_depth = getInput<double>("target_depth");
  auto depth_tolerance = getInput<double>("depth_tolerance");
  auto timeout = getInput<double>("timeout");
  auto surface_force_z = getInput<double>("surface_force_z");

  Surface::Goal goal;
  goal.target_depth = target_depth ? target_depth.value() : 0.0;
  goal.depth_tolerance = depth_tolerance ?
    positiveOrDefault(depth_tolerance.value(), 0.10) : 0.10;
  goal.timeout = timeout ? positiveOrDefault(timeout.value(), 30.0) : 30.0;
  goal.surface_force_z = surface_force_z ? surface_force_z.value() : -40.0;

  if (goal.surface_force_z >= 0.0)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] SurfaceAction requires surface_force_z to be negative");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(goal.timeout)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface action server is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = rclcpp_action::Client<Surface>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [ros_node](
      GoalHandleSurface::SharedPtr,
      const std::shared_ptr<const Surface::Feedback> feedback)
    {
      RCLCPP_DEBUG(
        ros_node->get_logger(),
        "[sura_bt] Surface feedback: current_depth=%.3f depth_error=%.3f time_remaining=%.3f state=%s",
        feedback->current_depth,
        feedback->depth_error,
        feedback->time_remaining,
        feedback->state.c_str());
    };

  auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
  const auto goal_handle_status = rclcpp::spin_until_future_complete(
    ros_node,
    goal_handle_future,
    std::chrono::duration<double>(goal.timeout));

  if (goal_handle_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface goal request timed out");
    return BT::NodeStatus::FAILURE;
  }

  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface goal was rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  const auto result_status = rclcpp::spin_until_future_complete(
    ros_node,
    result_future,
    std::chrono::duration<double>(goal.timeout + 1.0));

  if (result_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface result timed out");
    return BT::NodeStatus::FAILURE;
  }

  const auto wrapped_result = result_future.get();
  const auto result = wrapped_result.result;
  if (result)
  {
    setOutput("final_depth", result->final_depth);
    setOutput("message", result->message);
  }

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    result && result->success)
  {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] Surface action succeeded: %s final_depth=%.3f",
      result->message.c_str(),
      result->final_depth);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_bt] Surface action failed: %s",
    result ? result->message.c_str() : "missing result");
  return BT::NodeStatus::FAILURE;
}

std::string SurfaceAction::actionName(const BT::NodeConfiguration & config) const
{
  const auto robot_namespace =
    stripSlashes(config.blackboard->get<std::string>("robot_namespace"));

  if (robot_namespace.empty())
  {
    return "/actions/surface";
  }

  return "/" + robot_namespace + "/actions/surface";
}

}  // namespace sura_bt
