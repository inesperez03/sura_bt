#include "sura_bt/sura_actions_nodes.hpp"

#include <chrono>
#include <cmath>
#include <future>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

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

bool isFinite(double value)
{
  return std::isfinite(value);
}

}  // namespace

SurfaceAction::SurfaceAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  action_client_ =
    rclcpp_action::create_client<Surface>(
      getRosNode(config),
      actionName(config));
}

BT::PortsList SurfaceAction::providedPorts()
{
  return {
    BT::InputPort<double>("target_depth", "Target depth considered as the surface."),
    BT::InputPort<double>("depth_tolerance", "Acceptable distance from the target depth."),
    BT::InputPort<double>("timeout", "Maximum time allowed for the surfacing action to complete, in seconds."),
    BT::InputPort<double>("surface_force_z", "Vertical force used to drive the vehicle upward while surfacing."),
    BT::OutputPort<double>("final_depth", "Final depth reached after the surfacing action."),
    BT::OutputPort<std::string>("message", "Result message reported by the surfacing action.")
  };
}

const char * SurfaceAction::main_description()
{
  return "Brings the underwater vehicle to the surface.";
}

BT::NodeStatus SurfaceAction::onStart()
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
  active_goal_timeout_ = goal.timeout;

  if (goal.surface_force_z >= 0.0)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] SurfaceAction requires surface_force_z to be negative");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(1.0)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface action server is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = rclcpp_action::Client<Surface>::SendGoalOptions();
  resetGoalState(true);
  const uint64_t goal_id = active_goal_id_;

  send_goal_options.goal_response_callback =
    [this, goal_id](GoalHandleSurface::SharedPtr goal_handle)
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_id != active_goal_id_)
      {
        if (goal_handle)
        {
          (void)action_client_->async_cancel_goal(goal_handle);
        }
        return;
      }

      goal_response_received_ = true;

      if (!goal_handle)
      {
        goal_rejected_ = true;
        return;
      }

      if (cancel_requested_)
      {
        (void)action_client_->async_cancel_goal(goal_handle);
        return;
      }

      goal_handle_ = goal_handle;
    };

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

  send_goal_options.result_callback =
    [this, goal_id](const WrappedResult & wrapped_result)
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!goal_active_ || goal_id != active_goal_id_)
      {
        return;
      }
      wrapped_result_ = wrapped_result;
      result_received_ = true;
    };

  (void)action_client_->async_send_goal(goal, send_goal_options);
  active_goal_start_time_ = ros_node->now();

  RCLCPP_INFO(
    ros_node->get_logger(),
    "[sura_bt] Surface goal sent: target_depth=%.3f tolerance=%.3f timeout=%.3f surface_force_z=%.3f",
    goal.target_depth,
    goal.depth_tolerance,
    goal.timeout,
    goal.surface_force_z);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurfaceAction::onRunning()
{
  const auto ros_node = getRosNode(config());

  if (
    active_goal_timeout_ > 0.0 &&
    (ros_node->now() - active_goal_start_time_).seconds() > active_goal_timeout_ + 1.0)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface timed out in BT");
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      cancel_requested_ = true;
      if (goal_handle_)
      {
        (void)action_client_->async_cancel_goal(goal_handle_);
      }
    }
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }

  WrappedResult wrapped_result;
  bool goal_rejected = false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_rejected = goal_rejected_;

    if (!goal_rejected && (!goal_response_received_ || !result_received_))
    {
      return BT::NodeStatus::RUNNING;
    }

    if (!goal_rejected)
    {
      wrapped_result = wrapped_result_;
    }
  }

  if (goal_rejected)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Surface goal was rejected");
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }

  resetGoalState(false);
  return handleResult(wrapped_result);
}

void SurfaceAction::onHalted()
{
  const auto ros_node = getRosNode(config());
  GoalHandleSurface::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    cancel_requested_ = true;
    goal_active_ = false;
    active_goal_id_ = 0;
    goal_handle = goal_handle_;
  }

  if (goal_handle)
  {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] Surface halted: canceling active goal");
    (void)action_client_->async_cancel_goal(goal_handle);
  }
}

BT::NodeStatus SurfaceAction::handleResult(const WrappedResult & wrapped_result)
{
  const auto ros_node = getRosNode(config());
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

void SurfaceAction::resetGoalState(bool active)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  goal_active_ = active;
  active_goal_id_ = active ? ++next_goal_id_ : 0;
  goal_response_received_ = false;
  goal_rejected_ = false;
  result_received_ = false;
  cancel_requested_ = false;
  goal_handle_.reset();
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

GoToPoseAction::GoToPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  action_client_ =
    rclcpp_action::create_client<GoToPose>(
      getRosNode(config),
      actionName(config));
}

BT::PortsList GoToPoseAction::providedPorts()
{
  return {
    BT::InputPort<double>("x", "Target X position in the selected frame."),
    BT::InputPort<double>("y", "Target Y position in the selected frame."),
    BT::InputPort<double>("z", "Target Z position in the selected frame."),
    BT::InputPort<double>("yaw", "Target yaw angle in radians."),
    BT::InputPort<bool>("holonomic", false, "Whether to allow holonomic XY motion toward the target pose."),
    BT::InputPort<double>("timeout", 120.0, "Maximum time allowed to reach the target pose, in seconds."),
    BT::InputPort<std::string>("frame_id", "world_ned", "Coordinate frame used for the target pose."),
    BT::OutputPort<std::string>("message", "Result message reported by the go-to-pose action.")
  };
}

const char * GoToPoseAction::main_description()
{
  return "Moves the underwater vehicle to a requested target pose.";
}

BT::NodeStatus GoToPoseAction::onStart()
{
  const auto ros_node = getRosNode(config());

  const auto x = getInput<double>("x");
  const auto y = getInput<double>("y");
  const auto z = getInput<double>("z");
  const auto yaw = getInput<double>("yaw");

  if (!x || !y || !z || !yaw ||
    !isFinite(x.value()) || !isFinite(y.value()) ||
    !isFinite(z.value()) || !isFinite(yaw.value()))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] GoToPoseAction requires finite x, y, z and yaw inputs");
    return BT::NodeStatus::FAILURE;
  }

  active_goal_timeout_ =
    positiveOrDefault(getInput<double>("timeout").value_or(120.0), 120.0);

  if (!prepareLifecycle(ros_node, 10.0))
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(10.0)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] GoToPose action server is not available");
    return BT::NodeStatus::FAILURE;
  }

  GoToPose::Goal goal;
  goal.target_pose.header.stamp = ros_node->now();
  goal.target_pose.header.frame_id =
    getInput<std::string>("frame_id").value_or("world_ned");
  goal.target_pose.pose.position.x = x.value();
  goal.target_pose.pose.position.y = y.value();
  goal.target_pose.pose.position.z = z.value();
  goal.target_pose.pose.orientation.z = std::sin(yaw.value() * 0.5);
  goal.target_pose.pose.orientation.w = std::cos(yaw.value() * 0.5);
  goal.target_yaw = yaw.value();
  goal.holonomic = getInput<bool>("holonomic").value_or(false);
  goal.timeout = active_goal_timeout_;

  resetGoalState(true);
  const uint64_t goal_id = active_goal_id_;

  auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, goal_id](GoalHandleGoToPose::SharedPtr goal_handle)
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_id != active_goal_id_)
      {
        if (goal_handle)
        {
          (void)action_client_->async_cancel_goal(goal_handle);
        }
        return;
      }

      goal_response_received_ = true;

      if (!goal_handle)
      {
        goal_rejected_ = true;
        return;
      }

      if (cancel_requested_)
      {
        (void)action_client_->async_cancel_goal(goal_handle);
        return;
      }

      goal_handle_ = goal_handle;
    };

  send_goal_options.feedback_callback =
    [ros_node](
      GoalHandleGoToPose::SharedPtr,
      const std::shared_ptr<const GoToPose::Feedback> feedback)
    {
      RCLCPP_DEBUG(
        ros_node->get_logger(),
        "[sura_bt] GoToPose feedback: current=(%.3f, %.3f, %.3f) distance=%.3f yaw_error=%.3f state=%s",
        feedback->current_x,
        feedback->current_y,
        feedback->current_z,
        feedback->distance_to_goal,
        feedback->yaw_error,
        feedback->state.c_str());
    };

  send_goal_options.result_callback =
    [this, goal_id](const WrappedResult & wrapped_result)
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!goal_active_ || goal_id != active_goal_id_)
      {
        return;
      }
      wrapped_result_ = wrapped_result;
      result_received_ = true;
    };

  (void)action_client_->async_send_goal(goal, send_goal_options);
  active_goal_start_time_ = ros_node->now();

  RCLCPP_INFO(
    ros_node->get_logger(),
    "[sura_bt] GoToPose goal sent: target=(%.3f, %.3f, %.3f) yaw=%.3f holonomic=%s",
    x.value(),
    y.value(),
    z.value(),
    yaw.value(),
    goal.holonomic ? "true" : "false");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPoseAction::onRunning()
{
  const auto ros_node = getRosNode(config());

  if (
    active_goal_timeout_ > 0.0 &&
    (ros_node->now() - active_goal_start_time_).seconds() > active_goal_timeout_ + 1.0)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] GoToPose timed out in BT");
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      cancel_requested_ = true;
      if (goal_handle_)
      {
        (void)action_client_->async_cancel_goal(goal_handle_);
      }
    }
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }

  WrappedResult wrapped_result;
  bool goal_rejected = false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_rejected = goal_rejected_;

    if (!goal_rejected && (!goal_response_received_ || !result_received_))
    {
      return BT::NodeStatus::RUNNING;
    }

    if (!goal_rejected)
    {
      wrapped_result = wrapped_result_;
    }
  }

  if (goal_rejected)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] GoToPose goal was rejected");
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }

  resetGoalState(false);
  return handleResult(wrapped_result);
}

void GoToPoseAction::onHalted()
{
  const auto ros_node = getRosNode(config());
  GoalHandleGoToPose::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    cancel_requested_ = true;
    goal_active_ = false;
    active_goal_id_ = 0;
    goal_handle = goal_handle_;
  }

  if (goal_handle)
  {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] GoToPose halted: canceling active goal");
    (void)action_client_->async_cancel_goal(goal_handle);
  }

  stopLifecycle(ros_node, 1.0);
}

BT::NodeStatus GoToPoseAction::handleResult(const WrappedResult & wrapped_result)
{
  const auto ros_node = getRosNode(config());
  const auto result = wrapped_result.result;
  if (result)
  {
    setOutput("message", result->message);
  }

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    result && result->success)
  {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] GoToPose action succeeded: %s",
      result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_bt] GoToPose action failed: %s",
    result ? result->message.c_str() : "missing result");
  return BT::NodeStatus::FAILURE;
}

void GoToPoseAction::resetGoalState(bool active)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  goal_active_ = active;
  active_goal_id_ = active ? ++next_goal_id_ : 0;
  goal_response_received_ = false;
  goal_rejected_ = false;
  result_received_ = false;
  cancel_requested_ = false;
  goal_handle_.reset();
}

std::string GoToPoseAction::actionName(const BT::NodeConfiguration & config) const
{
  const auto robot_namespace =
    stripSlashes(config.blackboard->get<std::string>("robot_namespace"));

  if (robot_namespace.empty())
  {
    return "/actions/go_to_pose";
  }

  return "/" + robot_namespace + "/actions/go_to_pose";
}

std::string GoToPoseAction::lifecycleNodeName(const BT::NodeConfiguration &) const
{
  return "/go_to_pose_lifecycle_action_node";
}

bool GoToPoseAction::prepareLifecycle(
  const rclcpp::Node::SharedPtr & ros_node,
  double timeout_sec)
{
  const auto lifecycle_node = lifecycleNodeName(config());
  uint8_t state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

  if (!getLifecycleState(ros_node, lifecycle_node, state_id, timeout_sec))
  {
    return false;
  }

  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    if (!changeLifecycleState(
        ros_node,
        lifecycle_node,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout_sec))
    {
      return false;
    }

    if (!getLifecycleState(ros_node, lifecycle_node, state_id, timeout_sec))
    {
      return false;
    }
  }

  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    if (!changeLifecycleState(
        ros_node,
        lifecycle_node,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout_sec))
    {
      return false;
    }

    if (!getLifecycleState(ros_node, lifecycle_node, state_id, timeout_sec))
    {
      return false;
    }
  }

  if (state_id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] GoToPose lifecycle node is not active; state_id=%u",
      state_id);
    return false;
  }

  return true;
}

bool GoToPoseAction::getLifecycleState(
  const rclcpp::Node::SharedPtr & ros_node,
  const std::string & lifecycle_node,
  uint8_t & state_id,
  double timeout_sec)
{
  auto client = ros_node->create_client<GetState>(lifecycle_node + "/get_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Lifecycle get_state service is not available: %s/get_state",
      lifecycle_node.c_str());
    return false;
  }

  auto request = std::make_shared<GetState::Request>();
  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(
    ros_node,
    future,
    std::chrono::duration<double>(timeout_sec));

  if (status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Lifecycle get_state request timed out: %s",
      lifecycle_node.c_str());
    return false;
  }

  state_id = future.get()->current_state.id;
  return true;
}

bool GoToPoseAction::changeLifecycleState(
  const rclcpp::Node::SharedPtr & ros_node,
  const std::string & lifecycle_node,
  uint8_t transition_id,
  double timeout_sec)
{
  auto client = ros_node->create_client<ChangeState>(lifecycle_node + "/change_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Lifecycle change_state service is not available: %s/change_state",
      lifecycle_node.c_str());
    return false;
  }

  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition_id;
  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(
    ros_node,
    future,
    std::chrono::duration<double>(timeout_sec));

  if (status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Lifecycle change_state request timed out: %s",
      lifecycle_node.c_str());
    return false;
  }

  if (!future.get()->success)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Lifecycle transition %u failed: %s",
      transition_id,
      lifecycle_node.c_str());
    return false;
  }

  return true;
}

void GoToPoseAction::stopLifecycle(
  const rclcpp::Node::SharedPtr & ros_node,
  double timeout_sec)
{
  const auto lifecycle_node = lifecycleNodeName(config());
  uint8_t state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

  if (!getLifecycleState(ros_node, lifecycle_node, state_id, timeout_sec))
  {
    return;
  }

  if (state_id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return;
  }

  (void)changeLifecycleState(
    ros_node,
    lifecycle_node,
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
    timeout_sec);
}

}  // namespace sura_bt
