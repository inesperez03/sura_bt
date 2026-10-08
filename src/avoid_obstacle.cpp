#include "sura_bt/avoid_obstacle.hpp"
#include "sura_bt/mission_failure_reasons.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sura_safety/diagnostics_monitor.hpp"

namespace sura_bt
{
namespace
{
std::string stripSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {value.erase(value.begin());}
  while (!value.empty() && value.back() == '/') {value.pop_back();}
  return value;
}

bool isObstacleError(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  return status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
    status.message == "Front obstacle critically close";
}

bool isObstacleWarning(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  return status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
    status.message == "Front obstacle close";
}
}  // namespace

AvoidObstacle::AvoidObstacle(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  monitor_ = config.blackboard->get<std::shared_ptr<sura_safety::DiagnosticsMonitor>>(
    "diagnostics_monitor");
}

BT::PortsList AvoidObstacle::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Robot that executes the lateral maneuver."),
    BT::InputPort<std::string>("side", "right", "Lateral direction: left or right."),
    BT::InputPort<double>("lateral_speed", 0.2, "Positive lateral speed in metres per second."),
    BT::OutputPort<std::string>("message", "Reason for success or failure.")
  };
}

const char * AvoidObstacle::main_description()
{
  return "Moves an underwater robot sideways while Navigation/FrontObstacle reports a front "
    "obstacle. Succeeds when the diagnostic becomes OK; independent of the safety tree.";
}

BT::NodeStatus AvoidObstacle::onStart()
{
  const auto robot = getInput<std::string>("robot_namespace");
  robot_namespace_ = robot ? stripSlashes(robot.value()) : "";
  if (robot_namespace_.empty()) {
    return finish(BT::NodeStatus::FAILURE, "robot_namespace is required");
  }
  std::string family;
  if (!config().blackboard->get("robot_family_" + robot_namespace_, family) ||
    family != "underwater")
  {
    return finish(BT::NodeStatus::FAILURE, "robot must have an underwater profile");
  }
  std::vector<std::string> axes;
  if (!config().blackboard->get("robot_available_axes_" + robot_namespace_, axes) ||
    std::find(axes.begin(), axes.end(), "sway") == axes.end())
  {
    return finish(BT::NodeStatus::FAILURE, "robot profile does not confirm sway control");
  }

  const auto side = getInput<std::string>("side").value_or("right");
  const auto speed = getInput<double>("lateral_speed").value_or(0.2);
  if (side != "left" && side != "right") {
    return finish(BT::NodeStatus::FAILURE, "side must be left or right");
  }
  if (!std::isfinite(speed) || speed <= 0.0)
  {
    return finish(BT::NodeStatus::FAILURE, "lateral_speed must be positive and finite");
  }
  // The controller's body frame is forward-right-down, so positive Y is right.
  lateral_velocity_ = side == "right" ? speed : -speed;

  const auto diagnostic = monitor_->getStatus(
    "/" + robot_namespace_ + "/Navigation/FrontObstacle");
  if (!diagnostic || diagnostic->level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
    return finish(BT::NodeStatus::FAILURE, "front obstacle diagnostic is unavailable or stale");
  }
  if (diagnostic->level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return finish(BT::NodeStatus::SUCCESS, "front is already clear");
  }
  if (!isObstacleError(*diagnostic)) {
    return finish(BT::NodeStatus::FAILURE,
      "front obstacle diagnostic does not indicate a critically close obstacle");
  }

  requester_ = "avoid_obstacle_" + robot_namespace_ + "_" + name();
  mode_client_ = node_->create_client<Mode>(
    "/" + robot_namespace_ + "/control_manager/set_mode");
  velocity_pub_ = node_->create_publisher<VelocityCommand>(
    "/" + robot_namespace_ + "/controller/arbitrator/velocity",
    rclcpp::QoS(10));
  started_ = std::chrono::steady_clock::now();
  mode_requested_ = false;
  mode_ready_ = false;
  command_started_ = false;
  mode_future_ = {};
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AvoidObstacle::onRunning()
{
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_).count();
  const auto diagnostic = monitor_->getStatus(
    "/" + robot_namespace_ + "/Navigation/FrontObstacle");
  if (!diagnostic || diagnostic->level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
    return finish(BT::NodeStatus::FAILURE, "front obstacle diagnostic is unavailable or stale");
  }
  if (diagnostic->level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return finish(BT::NodeStatus::SUCCESS, "front obstacle cleared");
  }
  if (!isObstacleError(*diagnostic) && !isObstacleWarning(*diagnostic)) {
    return finish(BT::NodeStatus::FAILURE, "front obstacle diagnostic is invalid");
  }

  if (!mode_requested_) {
    if (!mode_client_->service_is_ready()) {
      if (elapsed >= 5.0) {
        return finish(BT::NodeStatus::FAILURE, "control mode service is unavailable");
      }
      return BT::NodeStatus::RUNNING;
    }
    auto request = std::make_shared<Mode::Request>();
    request->mode = Mode::Request::BODY_VELOCITY;
    request->reason = "AvoidObstacle";
    mode_future_ = mode_client_->async_send_request(request).share();
    mode_requested_ = true;
  }
  if (!mode_ready_) {
    if (mode_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (elapsed >= 5.0) {
        return finish(BT::NodeStatus::FAILURE, "control mode request timed out");
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!mode_future_.get()->success) {
      return finish(BT::NodeStatus::FAILURE, "BODY_VELOCITY control mode was rejected");
    }
    mode_ready_ = true;
  }

  publishVelocity(lateral_velocity_);
  command_started_ = true;
  return BT::NodeStatus::RUNNING;
}

void AvoidObstacle::onHalted()
{
  stopMotion();
}

BT::NodeStatus AvoidObstacle::finish(BT::NodeStatus status, const std::string & message)
{
  stopMotion();
  setOutput("message", message);
  if (status == BT::NodeStatus::FAILURE) {
    recordMissionFailure(config(), name(), message);
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] AvoidObstacle: %s", message.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[sura_bt] AvoidObstacle: %s", message.c_str());
  }
  return status;
}

void AvoidObstacle::publishVelocity(double lateral_velocity)
{
  VelocityCommand command;
  command.header.stamp = node_->now();
  command.requester = requester_;
  command.controller = "body_velocity";
  command.priority = 70;
  command.velocity.linear.y = lateral_velocity;
  velocity_pub_->publish(command);
}

void AvoidObstacle::stopMotion()
{
  if (command_started_) {
    publishVelocity(0.0);
    command_started_ = false;
  }
  mode_future_ = {};
  mode_requested_ = false;
  mode_ready_ = false;
}

}  // namespace sura_bt
