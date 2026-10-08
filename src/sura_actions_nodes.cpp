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

std::string requiredRobotNamespace(const BT::TreeNode & node)
{
  const auto robot_namespace = node.getInput<std::string>("robot_namespace");
  if (!robot_namespace || stripSlashes(robot_namespace.value()).empty())
  {
    throw BT::RuntimeError(node.name(), " requires a non-empty robot_namespace input");
  }
  return stripSlashes(robot_namespace.value());
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
    BT::InputPort<std::string>("robot_namespace", "{robot_namespace}", "Robot namespace that receives the surface action; defaults to the configured robot."),
    BT::InputPort<double>("target_depth", 0.0, "Target depth considered as the surface."),
    BT::InputPort<double>("depth_tolerance", 0.10, "Acceptable distance from the target depth."),
    BT::InputPort<double>("timeout", 30.0, "Maximum time allowed for the surfacing action to complete, in seconds."),
    BT::InputPort<double>("surface_force_z", -40.0, "Vertical force used to drive the vehicle upward while surfacing."),
    BT::OutputPort<double>("final_depth", "Final depth reached after the surfacing action."),
    BT::OutputPort<std::string>("message", "Result message reported by the surfacing action.")
  };
}

const char * SurfaceAction::main_description()
{
  return "Compatibilidad: vehículos submarinos con servidor de acción /<robot_namespace>/actions/surface, "
    "sensor de profundidad y control de fuerza vertical. No está implementada para vehículos de superficie.";
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

std::string SurfaceAction::actionName(const BT::NodeConfiguration &) const
{
  const auto robot_namespace = requiredRobotNamespace(*this);

  if (robot_namespace.empty())
  {
    return "/actions/surface";
  }

  return "/" + robot_namespace + "/actions/surface";
}

GoToPoseUnderwaterBackend::GoToPoseUnderwaterBackend(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  action_client_ =
    rclcpp_action::create_client<GoToPose>(
      getRosNode(config),
      actionName(config));
}

BT::PortsList GoToPoseUnderwaterBackend::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Robot namespace that receives the go-to-pose action."),
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

const char * GoToPoseUnderwaterBackend::main_description()
{
  return "Compatibilidad: vehículos submarinos que ejecuten su servidor "
    "/<robot_namespace>/actions/go_to_pose activo y la configuración de control correspondiente. "
    "El árbol usa GoToPoseAction para ambas familias y selecciona el servidor desde el perfil.";
}

BT::NodeStatus GoToPoseUnderwaterBackend::onStart()
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

BT::NodeStatus GoToPoseUnderwaterBackend::onRunning()
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

void GoToPoseUnderwaterBackend::onHalted()
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

BT::NodeStatus GoToPoseUnderwaterBackend::handleResult(const WrappedResult & wrapped_result)
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

void GoToPoseUnderwaterBackend::resetGoalState(bool active)
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

std::string GoToPoseUnderwaterBackend::actionName(const BT::NodeConfiguration &) const
{
  const auto robot_namespace = requiredRobotNamespace(*this);

  if (robot_namespace.empty())
  {
    return "/actions/go_to_pose";
  }

  return "/" + robot_namespace + "/actions/go_to_pose";
}

std::string GoToPoseUnderwaterBackend::lifecycleNodeName(const BT::NodeConfiguration &) const
{
  // actions.launch.py keeps the lifecycle node in the root namespace; only its
  // action endpoint is namespaced per robot.
  return "/go_to_pose_lifecycle_action_node";
}

bool GoToPoseUnderwaterBackend::prepareLifecycle(
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

bool GoToPoseUnderwaterBackend::getLifecycleState(
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

bool GoToPoseUnderwaterBackend::changeLifecycleState(
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

void GoToPoseUnderwaterBackend::stopLifecycle(
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



namespace
{

std::string stripNamespaceSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {value.erase(value.begin());}
  while (!value.empty() && value.back() == '/') {value.pop_back();}
  return value;
}

std::string requiredUsvRobotNamespace(const BT::TreeNode & node)
{
  const auto value = node.getInput<std::string>("robot_namespace");
  if (!value || stripNamespaceSlashes(value.value()).empty()) {
    throw BT::RuntimeError(node.name(), " requires a non-empty robot_namespace input");
  }
  return stripNamespaceSlashes(value.value());
}

rclcpp::Node::SharedPtr getUsvRosNode(const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

bool finiteValue(double value)
{
  return std::isfinite(value);
}

}  // namespace

GoToPoseSurfaceBackend::GoToPoseSurfaceBackend(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  action_client_ = rclcpp_action::create_client<GoToPoseUsv>(
    getUsvRosNode(config), actionName(config));
}

BT::PortsList GoToPoseSurfaceBackend::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Surface robot namespace receiving the pose goal."),
    BT::InputPort<double>("x", "Target X position in the configured planar frame."),
    BT::InputPort<double>("y", "Target Y position in the configured planar frame."),
    BT::InputPort<double>("yaw", "Target yaw angle in radians."),
    BT::InputPort<double>("max_forward_speed", 0.0, "Maximum forward speed; zero uses the robot default."),
    BT::InputPort<double>("max_yaw_rate", 0.0, "Maximum yaw rate; zero uses the robot default."),
    BT::InputPort<double>("position_tolerance", 0.0, "Position tolerance; zero uses the robot default."),
    BT::InputPort<double>("yaw_tolerance", 0.0, "Yaw tolerance; zero uses the robot default."),
    BT::InputPort<double>("slowdown_distance", 0.0, "Slowdown distance; zero uses the robot default."),
    BT::InputPort<double>("timeout", 120.0, "Maximum time allowed to reach the target, in seconds."),
    BT::OutputPort<std::string>("message", "Result message reported by the surface GoToPose action.")
  };
}

const char * GoToPoseSurfaceBackend::main_description()
{
  return "Moves a surface vehicle to a planar target pose using the GoToPoseUsv action. "
    "The action server selects BODY_VELOCITY control mode itself; do not precede this node "
    "with ActivateControllers.";
}

BT::NodeStatus GoToPoseSurfaceBackend::onStart()
{
  const auto node = getUsvRosNode(config());
  const auto x = getInput<double>("x");
  const auto y = getInput<double>("y");
  const auto yaw = getInput<double>("yaw");
  if (!x || !y || !yaw || !finiteValue(x.value()) || !finiteValue(y.value()) ||
    !finiteValue(yaw.value()))
  {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Surface GoToPose backend requires finite x, y and yaw inputs");
    return BT::NodeStatus::FAILURE;
  }

  active_goal_timeout_ = positiveOrDefault(getInput<double>("timeout").value_or(120.0), 120.0);
  if (!prepareLifecycle(node, 10.0)) {return BT::NodeStatus::FAILURE;}
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(10.0))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] USV GoToPose action server is not available");
    return BT::NodeStatus::FAILURE;
  }

  GoToPoseUsv::Goal goal;
  goal.target_pose.x = x.value();
  goal.target_pose.y = y.value();
  goal.target_pose.theta = yaw.value();
  goal.use_planned_pose = false;
  goal.max_forward_speed = getInput<double>("max_forward_speed").value_or(0.0);
  goal.max_yaw_rate = getInput<double>("max_yaw_rate").value_or(0.0);
  goal.position_tolerance = getInput<double>("position_tolerance").value_or(0.0);
  goal.yaw_tolerance = getInput<double>("yaw_tolerance").value_or(0.0);
  goal.slowdown_distance = getInput<double>("slowdown_distance").value_or(0.0);
  goal.timeout = active_goal_timeout_;

  resetGoalState(true);
  const auto goal_id = active_goal_id_;
  auto options = rclcpp_action::Client<GoToPoseUsv>::SendGoalOptions();
  options.goal_response_callback = [this, goal_id](GoalHandleGoToPoseUsv::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_id != active_goal_id_) {return;}
      goal_response_received_ = true;
      if (!handle) {goal_rejected_ = true; return;}
      if (cancel_requested_) {(void)action_client_->async_cancel_goal(handle); return;}
      goal_handle_ = handle;
    };
  options.feedback_callback = [node](
    GoalHandleGoToPoseUsv::SharedPtr,
    const std::shared_ptr<const GoToPoseUsv::Feedback> feedback)
    {
      RCLCPP_DEBUG(
        node->get_logger(), "[sura_bt] GoToPoseUsv feedback: current=(%.3f, %.3f) "
        "distance=%.3f yaw_error=%.3f state=%s", feedback->current_x, feedback->current_y,
        feedback->distance_to_goal, feedback->yaw_error, feedback->state.c_str());
    };
  options.result_callback = [this, goal_id](const WrappedResult & result) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!goal_active_ || goal_id != active_goal_id_) {return;}
      wrapped_result_ = result;
      result_received_ = true;
    };

  (void)action_client_->async_send_goal(goal, options);
  active_goal_start_time_ = node->now();
  RCLCPP_INFO(
    node->get_logger(), "[sura_bt] GoToPoseUsv goal sent: target=(%.3f, %.3f) yaw=%.3f",
    x.value(), y.value(), yaw.value());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPoseSurfaceBackend::onRunning()
{
  const auto node = getUsvRosNode(config());
  if (active_goal_timeout_ > 0.0 &&
    (node->now() - active_goal_start_time_).seconds() > active_goal_timeout_ + 1.0)
  {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] GoToPoseUsv timed out in BT");
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      cancel_requested_ = true;
      if (goal_handle_) {(void)action_client_->async_cancel_goal(goal_handle_);}
    }
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }

  WrappedResult result;
  bool rejected = false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    rejected = goal_rejected_;
    if (!rejected && (!goal_response_received_ || !result_received_)) {
      return BT::NodeStatus::RUNNING;
    }
    if (!rejected) {result = wrapped_result_;}
  }
  if (rejected) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] GoToPoseUsv goal was rejected");
    resetGoalState(false);
    return BT::NodeStatus::FAILURE;
  }
  resetGoalState(false);
  return handleResult(result);
}

void GoToPoseSurfaceBackend::onHalted()
{
  const auto node = getUsvRosNode(config());
  GoalHandleGoToPoseUsv::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    cancel_requested_ = true;
    goal_active_ = false;
    active_goal_id_ = 0;
    handle = goal_handle_;
  }
  if (handle) {(void)action_client_->async_cancel_goal(handle);}
  stopLifecycle(node, 1.0);
}

BT::NodeStatus GoToPoseSurfaceBackend::handleResult(const WrappedResult & wrapped_result)
{
  const auto result = wrapped_result.result;
  if (result) {setOutput("message", result->message);}
  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && result && result->success) {
    RCLCPP_INFO(getUsvRosNode(config())->get_logger(), "[sura_bt] GoToPoseUsv succeeded: %s",
      result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(getUsvRosNode(config())->get_logger(), "[sura_bt] GoToPoseUsv failed: %s",
    result ? result->message.c_str() : "missing result");
  return BT::NodeStatus::FAILURE;
}

void GoToPoseSurfaceBackend::resetGoalState(bool active)
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

std::string GoToPoseSurfaceBackend::actionName(const BT::NodeConfiguration &) const
{
  return "/" + requiredUsvRobotNamespace(*this) + "/actions/go_to_pose";
}

std::string GoToPoseSurfaceBackend::lifecycleNodeName(const BT::NodeConfiguration &) const
{
  // actions.launch.py keeps lifecycle node names at the root namespace while
  // the action endpoint itself is explicitly namespaced by robot_namespace.
  return "/go_to_pose_usv_lifecycle_action_node";
}

bool GoToPoseSurfaceBackend::prepareLifecycle(
  const rclcpp::Node::SharedPtr & node, double timeout_sec)
{
  const auto lifecycle_node = lifecycleNodeName(config());
  uint8_t state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  if (!getLifecycleState(node, lifecycle_node, state, timeout_sec)) {return false;}
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    if (!changeLifecycleState(node, lifecycle_node,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout_sec) ||
      !getLifecycleState(node, lifecycle_node, state, timeout_sec)) {return false;}
  }
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!changeLifecycleState(node, lifecycle_node,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout_sec) ||
      !getLifecycleState(node, lifecycle_node, state, timeout_sec)) {return false;}
  }
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] USV GoToPose lifecycle node is not active; state=%u", state);
    return false;
  }
  return true;
}

bool GoToPoseSurfaceBackend::getLifecycleState(
  const rclcpp::Node::SharedPtr & node,
  const std::string & lifecycle_node,
  uint8_t & state,
  double timeout_sec)
{
  auto client = node->create_client<GetState>(lifecycle_node + "/get_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Lifecycle get_state unavailable: %s",
      lifecycle_node.c_str());
    return false;
  }
  auto future = client->async_send_request(std::make_shared<GetState::Request>());
  if (rclcpp::spin_until_future_complete(node, future,
      std::chrono::duration<double>(timeout_sec)) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Lifecycle get_state timed out: %s",
      lifecycle_node.c_str());
    return false;
  }
  state = future.get()->current_state.id;
  return true;
}

bool GoToPoseSurfaceBackend::changeLifecycleState(
  const rclcpp::Node::SharedPtr & node,
  const std::string & lifecycle_node,
  uint8_t transition_id,
  double timeout_sec)
{
  auto client = node->create_client<ChangeState>(lifecycle_node + "/change_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Lifecycle change_state unavailable: %s",
      lifecycle_node.c_str());
    return false;
  }
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition_id;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future,
      std::chrono::duration<double>(timeout_sec)) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Lifecycle transition timed out: %s",
      lifecycle_node.c_str());
    return false;
  }
  if (!future.get()->success) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] Lifecycle transition %u failed: %s",
      transition_id, lifecycle_node.c_str());
    return false;
  }
  return true;
}

void GoToPoseSurfaceBackend::stopLifecycle(
  const rclcpp::Node::SharedPtr & node, double timeout_sec)
{
  const auto lifecycle_node = lifecycleNodeName(config());
  uint8_t state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  if (!getLifecycleState(node, lifecycle_node, state, timeout_sec) ||
    state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {return;}
  (void)changeLifecycleState(node, lifecycle_node,
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, timeout_sec);
}


GoToPoseAction::GoToPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList GoToPoseAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Robot that receives the pose goal."),
    BT::InputPort<double>("x", "Target X position."),
    BT::InputPort<double>("y", "Target Y position."),
    BT::InputPort<double>("z", "Target Z position; used by underwater robots."),
    BT::InputPort<double>("yaw", "Target yaw angle in radians."),
    BT::InputPort<bool>("holonomic", false, "Whether underwater XY motion may be holonomic."),
    BT::InputPort<std::string>("frame_id", "world_ned", "Target frame for underwater robots."),
    BT::InputPort<double>("max_forward_speed", 0.0, "Surface forward speed; zero uses robot default."),
    BT::InputPort<double>("max_yaw_rate", 0.0, "Surface yaw rate; zero uses robot default."),
    BT::InputPort<double>("position_tolerance", 0.0, "Surface position tolerance; zero uses robot default."),
    BT::InputPort<double>("yaw_tolerance", 0.0, "Surface yaw tolerance; zero uses robot default."),
    BT::InputPort<double>("slowdown_distance", 0.0, "Surface slowdown distance; zero uses robot default."),
    BT::InputPort<double>("timeout", 120.0, "Maximum time allowed to reach the target, in seconds."),
    BT::OutputPort<std::string>("message", "Result reported by the robot's GoToPose action.")
  };
}

const char * GoToPoseAction::main_description()
{
  return "Moves the selected robot to a target pose. Selects the surface or underwater "
    "action server automatically from the robot family in its profile.";
}

BT::NodeStatus GoToPoseAction::onStart()
{
  const auto robot = getInput<std::string>("robot_namespace");
  if (!robot || robot->empty()) {
    RCLCPP_ERROR(getRosNode(config())->get_logger(),
      "[sura_bt] GoToPoseAction requires a non-empty robot_namespace input");
    return BT::NodeStatus::FAILURE;
  }

  const auto family_key = "robot_family_" + stripNamespaceSlashes(robot.value());
  std::string family;
  if (!config().blackboard->get(family_key, family)) {
    RCLCPP_ERROR(getRosNode(config())->get_logger(),
      "[sura_bt] No robot family is available in the profile for '%s'", robot->c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (family == "surface") {
    backend_ = std::make_unique<GoToPoseSurfaceBackend>(name(), config());
  } else if (family == "underwater") {
    backend_ = std::make_unique<GoToPoseUnderwaterBackend>(name(), config());
  } else {
    RCLCPP_ERROR(getRosNode(config())->get_logger(),
      "[sura_bt] Unsupported robot family '%s' for '%s'", family.c_str(), robot->c_str());
    return BT::NodeStatus::FAILURE;
  }
  return tickBackend();
}

BT::NodeStatus GoToPoseAction::onRunning()
{
  return backend_ ? tickBackend() : BT::NodeStatus::FAILURE;
}

BT::NodeStatus GoToPoseAction::tickBackend()
{
  const auto status = backend_->executeTick();
  if (status != BT::NodeStatus::RUNNING) {
    backend_.reset();
  }
  return status;
}

void GoToPoseAction::onHalted()
{
  if (backend_) {
    backend_->halt();
    backend_.reset();
  }
}

OrbitPointAction::OrbitPointAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList OrbitPointAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Underwater robot that receives the orbit goal."),
    BT::InputPort<double>("center_x", "Orbit center X in the server's configured frame."),
    BT::InputPort<double>("center_y", "Orbit center Y in the server's configured frame."),
    BT::InputPort<double>("center_z", "Orbit center Z in the server's configured frame."),
    BT::InputPort<double>("radius", "Orbit radius in metres."),
    BT::InputPort<bool>("use_planned_orbit", false,
      "Use the center and radius planned in RViz; explicit coordinates are then optional."),
    BT::InputPort<double>("timeout", 310.0,
      "BT watchdog in seconds; the action server has its own configured timeout."),
    BT::OutputPort<std::string>("message", "Result reported by the orbit action.")
  };
}

const char * OrbitPointAction::main_description()
{
  return "Makes an underwater robot orbit a point once using OrbitPoint. "
    "Configures and activates the lifecycle action server, then sends the goal. "
    "Requires an AUV with lateral velocity control; use_planned_orbit selects the RViz plan.";
}

BT::NodeStatus OrbitPointAction::onStart()
{
  const auto node = getRosNode(config());
  std::string robot_namespace;
  try {
    robot_namespace = requiredRobotNamespace(*this);
  } catch (const BT::RuntimeError & e) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  std::string family;
  if (!config().blackboard->get("robot_family_" + robot_namespace, family) ||
    family != "underwater")
  {
    RCLCPP_ERROR(node->get_logger(),
      "[sura_bt] OrbitPointAction requires an underwater robot profile for '%s'",
      robot_namespace.c_str());
    return BT::NodeStatus::FAILURE;
  }

  OrbitPoint::Goal goal;
  goal.use_planned_orbit = getInput<bool>("use_planned_orbit").value_or(false);
  if (!goal.use_planned_orbit) {
    const auto x = getInput<double>("center_x");
    const auto y = getInput<double>("center_y");
    const auto z = getInput<double>("center_z");
    const auto radius = getInput<double>("radius");
    if (!x || !y || !z || !radius || !isFinite(x.value()) ||
      !isFinite(y.value()) || !isFinite(z.value()) ||
      !isFinite(radius.value()) || radius.value() <= 0.0)
    {
      RCLCPP_ERROR(node->get_logger(),
        "[sura_bt] OrbitPointAction requires finite center_x, center_y, center_z and positive radius");
      return BT::NodeStatus::FAILURE;
    }
    goal.center.x = x.value();
    goal.center.y = y.value();
    goal.center.z = z.value();
    goal.radius = radius.value();
  }

  const double timeout = getInput<double>("timeout").value_or(310.0);
  if (!isFinite(timeout) || timeout <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPointAction requires a positive timeout");
    return BT::NodeStatus::FAILURE;
  }
  active_goal_timeout_ = timeout;

  if (!prepareLifecycle(node)) {return BT::NodeStatus::FAILURE;}
  action_client_ = rclcpp_action::create_client<OrbitPoint>(
    node, "/" + robot_namespace + "/actions/orbit_point");
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint action server is not available");
    stopLifecycle(node);
    return BT::NodeStatus::FAILURE;
  }

  resetGoalState();
  const auto goal_id = active_goal_id_;
  auto options = rclcpp_action::Client<OrbitPoint>::SendGoalOptions();
  options.goal_response_callback = [this, goal_id](GoalHandle::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_id != active_goal_id_ || cancel_requested_) {
        if (handle) {(void)action_client_->async_cancel_goal(handle);}
        return;
      }
      goal_response_received_ = true;
      if (!handle) {goal_rejected_ = true; return;}
      goal_handle_ = handle;
    };
  options.feedback_callback = [node](
    GoalHandle::SharedPtr, const std::shared_ptr<const OrbitPoint::Feedback> feedback) {
      RCLCPP_DEBUG(node->get_logger(),
        "[sura_bt] OrbitPoint feedback: progress=%.3f radial_error=%.3f yaw_error=%.3f state=%s",
        feedback->progress, feedback->radial_error, feedback->yaw_error,
        feedback->state.c_str());
    };
  options.result_callback = [this, goal_id](const WrappedResult & result) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_id != active_goal_id_) {return;}
      wrapped_result_ = result;
      result_received_ = true;
    };
  (void)action_client_->async_send_goal(goal, options);
  active_goal_start_time_ = node->now();
  RCLCPP_INFO(node->get_logger(), "[sura_bt] OrbitPoint goal sent to %s",
    robot_namespace.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OrbitPointAction::onRunning()
{
  const auto node = getRosNode(config());
  if ((node->now() - active_goal_start_time_).seconds() > active_goal_timeout_) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint timed out in BT");
    cancelGoal();
    stopLifecycle(node);
    return BT::NodeStatus::FAILURE;
  }

  WrappedResult result;
  bool rejected = false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    rejected = goal_rejected_;
    if (!rejected && (!goal_response_received_ || !result_received_)) {
      return BT::NodeStatus::RUNNING;
    }
    if (!rejected) {result = wrapped_result_;}
  }
  if (rejected) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint goal was rejected");
    cancelGoal();
    stopLifecycle(node);
    return BT::NodeStatus::FAILURE;
  }

  const auto action_result = result.result;
  if (action_result) {setOutput("message", action_result->message);}
  cancelGoal();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    action_result && action_result->success)
  {
    RCLCPP_INFO(node->get_logger(), "[sura_bt] OrbitPoint succeeded: %s",
      action_result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint failed: %s",
    action_result ? action_result->message.c_str() : "missing result");
  return BT::NodeStatus::FAILURE;
}

void OrbitPointAction::onHalted()
{
  cancelGoal();
  stopLifecycle(getRosNode(config()));
}

void OrbitPointAction::cancelGoal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  cancel_requested_ = true;
  active_goal_id_ = 0;
  if (goal_handle_ && action_client_ && !result_received_) {
    (void)action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
}

void OrbitPointAction::resetGoalState()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  active_goal_id_ = ++next_goal_id_;
  goal_handle_.reset();
  goal_response_received_ = false;
  goal_rejected_ = false;
  result_received_ = false;
  cancel_requested_ = false;
}

bool OrbitPointAction::prepareLifecycle(const rclcpp::Node::SharedPtr & node)
{
  uint8_t state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  if (!getLifecycleState(node, state)) {return false;}
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ||
      !getLifecycleState(node, state)) {return false;}
  }
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) ||
      !getLifecycleState(node, state)) {return false;}
  }
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint lifecycle state is %u", state);
    return false;
  }
  return true;
}

void OrbitPointAction::stopLifecycle(const rclcpp::Node::SharedPtr & node)
{
  uint8_t state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  if (getLifecycleState(node, state, 1.0) &&
    state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    (void)changeLifecycleState(
      node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 1.0);
  }
}

bool OrbitPointAction::getLifecycleState(
  const rclcpp::Node::SharedPtr & node, uint8_t & state, double timeout_sec)
{
  constexpr auto lifecycle_name = "/orbit_point_lifecycle_action_node";
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(
    std::string(lifecycle_name) + "/get_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint get_state service unavailable");
    return false;
  }
  auto future = client->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());
  if (rclcpp::spin_until_future_complete(
      node, future, std::chrono::duration<double>(timeout_sec)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint get_state timed out");
    return false;
  }
  state = future.get()->current_state.id;
  return true;
}

bool OrbitPointAction::changeLifecycleState(
  const rclcpp::Node::SharedPtr & node, uint8_t transition, double timeout_sec)
{
  constexpr auto lifecycle_name = "/orbit_point_lifecycle_action_node";
  auto client = node->create_client<lifecycle_msgs::srv::ChangeState>(
    std::string(lifecycle_name) + "/change_state");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint change_state service unavailable");
    return false;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      node, future, std::chrono::duration<double>(timeout_sec)) !=
    rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
    RCLCPP_ERROR(node->get_logger(), "[sura_bt] OrbitPoint lifecycle transition %u failed",
      transition);
    return false;
  }
  return true;
}

}  // namespace sura_bt
