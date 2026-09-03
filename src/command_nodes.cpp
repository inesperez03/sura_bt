#include "sura_bt/command_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>

#include "sura_safety/diagnostics_monitor.hpp"

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

std::string namespacedTopic(const std::string & robot_namespace, const std::string & suffix)
{
  const auto normalized_namespace = stripSlashes(robot_namespace);
  const auto normalized_suffix = stripSlashes(suffix);

  if (normalized_namespace.empty())
  {
    return "/" + normalized_suffix;
  }

  return "/" + normalized_namespace + "/" + normalized_suffix;
}

std::optional<double> parseDouble(const std::string & text)
{
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);

  if (end == text.c_str() || *end != '\0' || !std::isfinite(value))
  {
    return std::nullopt;
  }

  return value;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

rclcpp::Node::SharedPtr getRosNode(const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

std::shared_ptr<sura_safety::DiagnosticsMonitor> getMonitor(
  const BT::NodeConfiguration & config)
{
  return config.blackboard->get<std::shared_ptr<sura_safety::DiagnosticsMonitor>>(
    "diagnostics_monitor");
}

}  // namespace

SendWrench::SendWrench(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  const auto ros_node = getRosNode(config);
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");

  wrench_pub_ = ros_node->create_publisher<WrenchCommand>(
    namespacedTopic(robot_namespace, "controller/arbitrator/wrench"),
    rclcpp::SystemDefaultsQoS());
}

BT::PortsList SendWrench::providedPorts()
{
  return {
    BT::InputPort<double>("force_x", "Body-frame force command along the X axis."),
    BT::InputPort<double>("torque_x", "Body-frame torque command around the X axis."),
    BT::InputPort<double>("force_y", "Body-frame force command along the Y axis."),
    BT::InputPort<double>("torque_y", "Body-frame torque command around the Y axis."),
    BT::InputPort<double>("force_z", "Body-frame force command along the Z axis."),
    BT::InputPort<double>("torque_z", "Body-frame torque command around the Z axis."),
    BT::InputPort<int>("priority", "Priority of the wrench command from 1 to 100."),
    BT::InputPort<std::string>("reason", "Mission-level reason for sending this wrench command.")
  };
}

const char * SendWrench::main_description()
{
  return "Publishes a body-frame wrench command for the underwater vehicle.";
}

BT::NodeStatus SendWrench::tick()
{
  const auto ros_node = getRosNode(config());

  auto reason = getInput<std::string>("reason");
  if (!reason)
  {
    reason = "unknown";
  }

  auto priority = getInput<int>("priority");
  if (!priority)
  {
    priority = 85;
  }

  auto force_x = getInput<double>("force_x");
  if (!force_x)
  {
    force_x = 0.0;
  }

  auto torque_x = getInput<double>("torque_x");
  if (!torque_x)
  {
    torque_x = 0.0;
  }

  auto force_y = getInput<double>("force_y");
  if (!force_y)
  {
    force_y = 0.0;
  }

  auto torque_y = getInput<double>("torque_y");
  if (!torque_y)
  {
    torque_y = 0.0;
  }

  auto force_z = getInput<double>("force_z");
  if (!force_z)
  {
    force_z = 0.0;
  }

  auto torque_z = getInput<double>("torque_z");
  if (!torque_z)
  {
    torque_z = 0.0;
  }

  WrenchCommand msg;
  msg.header.stamp = ros_node->now();
  msg.requester = "sura_safety";
  msg.controller = "body_force";
  msg.priority = static_cast<uint8_t>(std::clamp(priority.value(), 1, 100));
  msg.wrench.force.x = force_x.value();
  msg.wrench.force.y = force_y.value();
  msg.wrench.force.z = force_z.value();
  msg.wrench.torque.x = torque_x.value();
  msg.wrench.torque.y = torque_y.value();
  msg.wrench.torque.z = torque_z.value();

  wrench_pub_->publish(msg);

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_bt] Action: send wrench. reason=%s controller=%s priority=%u force=(%.3f, %.3f, %.3f) torque=(%.3f, %.3f, %.3f)",
    reason.value().c_str(),
    msg.controller.c_str(),
    msg.priority,
    msg.wrench.force.x,
    msg.wrench.force.y,
    msg.wrench.force.z,
    msg.wrench.torque.x,
    msg.wrench.torque.y,
    msg.wrench.torque.z);

  return BT::NodeStatus::SUCCESS;
}

SendVelocity::SendVelocity(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  const auto ros_node = getRosNode(config);
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");

  velocity_pub_ = ros_node->create_publisher<VelocityCommand>(
    namespacedTopic(robot_namespace, "controller/arbitrator/velocity"),
    rclcpp::SystemDefaultsQoS());
}

BT::PortsList SendVelocity::providedPorts()
{
  return {
    BT::InputPort<double>("linear_x", "Body-frame linear velocity command along the X axis."),
    BT::InputPort<double>("angular_x", "Body-frame angular velocity command around the X axis."),
    BT::InputPort<double>("linear_y", "Body-frame linear velocity command along the Y axis."),
    BT::InputPort<double>("angular_y", "Body-frame angular velocity command around the Y axis."),
    BT::InputPort<double>("linear_z", "Body-frame linear velocity command along the Z axis."),
    BT::InputPort<double>("angular_z", "Body-frame angular velocity command around the Z axis."),
    BT::InputPort<int>("priority", "Priority of the velocity command from 1 to 100."),
    BT::InputPort<std::string>("reason", "Mission-level reason for sending this velocity command.")
  };
}

const char * SendVelocity::main_description()
{
  return "Publishes a body-frame velocity command for the underwater vehicle.";
}

BT::NodeStatus SendVelocity::tick()
{
  const auto ros_node = getRosNode(config());

  auto reason = getInput<std::string>("reason");
  if (!reason)
  {
    reason = "unknown";
  }

  auto priority = getInput<int>("priority");
  if (!priority)
  {
    priority = 85;
  }

  auto linear_x = getInput<double>("linear_x");
  if (!linear_x)
  {
    linear_x = 0.0;
  }

  auto angular_x = getInput<double>("angular_x");
  if (!angular_x)
  {
    angular_x = 0.0;
  }

  auto linear_y = getInput<double>("linear_y");
  if (!linear_y)
  {
    linear_y = 0.0;
  }

  auto angular_y = getInput<double>("angular_y");
  if (!angular_y)
  {
    angular_y = 0.0;
  }

  auto linear_z = getInput<double>("linear_z");
  if (!linear_z)
  {
    linear_z = 0.0;
  }

  auto angular_z = getInput<double>("angular_z");
  if (!angular_z)
  {
    angular_z = 0.0;
  }

  VelocityCommand msg;
  msg.header.stamp = ros_node->now();
  msg.requester = "sura_safety";
  msg.controller = "body_velocity";
  msg.priority = static_cast<uint8_t>(std::clamp(priority.value(), 1, 100));
  msg.velocity.linear.x = linear_x.value();
  msg.velocity.linear.y = linear_y.value();
  msg.velocity.linear.z = linear_z.value();
  msg.velocity.angular.x = angular_x.value();
  msg.velocity.angular.y = angular_y.value();
  msg.velocity.angular.z = angular_z.value();

  velocity_pub_->publish(msg);

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_bt] Action: send velocity. reason=%s controller=%s priority=%u linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)",
    reason.value().c_str(),
    msg.controller.c_str(),
    msg.priority,
    msg.velocity.linear.x,
    msg.velocity.linear.y,
    msg.velocity.linear.z,
    msg.velocity.angular.x,
    msg.velocity.angular.y,
    msg.velocity.angular.z);

  return BT::NodeStatus::SUCCESS;
}

ComputeAreaRecoveryForce::ComputeAreaRecoveryForce(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  const auto ros_node = getRosNode(config);
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");

  navigator_sub_ = ros_node->create_subscription<Navigator>(
    namespacedTopic(robot_namespace, "navigator/navigation"),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ComputeAreaRecoveryForce::navigatorCallback, this, std::placeholders::_1));
}

BT::PortsList ComputeAreaRecoveryForce::providedPorts()
{
  return {
    BT::InputPort<double>("force", "Magnitude of the recovery force to direct the vehicle back inside the allowed area."),
    BT::InputPort<std::string>("diagnostic_name", "Full diagnostic entry used to determine the area limit violation."),
    BT::InputPort<std::string>("diagnostic_suffix", "Diagnostic suffix used to locate the area limit diagnostic when no full name is provided."),
    BT::OutputPort<double>("output_x", "Computed body-frame force along the X axis."),
    BT::OutputPort<double>("output_y", "Computed body-frame force along the Y axis.")
  };
}

const char * ComputeAreaRecoveryForce::main_description()
{
  return "Computes a recovery force that drives the vehicle back inside the allowed operation area.";
}

void ComputeAreaRecoveryForce::navigatorCallback(const Navigator::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(navigator_mutex_);
  last_navigator_msg_ = msg;
}

BT::NodeStatus ComputeAreaRecoveryForce::tick()
{
  const auto monitor = getMonitor(config());
  auto diagnostic_name = getInput<std::string>("diagnostic_name");
  if (!diagnostic_name)
  {
    auto diagnostic_suffix = getInput<std::string>("diagnostic_suffix");
    if (!diagnostic_suffix)
    {
      diagnostic_suffix = "AreaLimit";
    }

    const std::string diagnostic_prefix =
      config().blackboard->get<std::string>("diagnostic_prefix");
    diagnostic_name =
      diagnostic_prefix + "/Navigation/ Navigation " + diagnostic_suffix.value();
  }

  const auto current_x = parseDouble(
    monitor->getValue(diagnostic_name.value(), "current_x"));
  const auto current_y = parseDouble(
    monitor->getValue(diagnostic_name.value(), "current_y"));
  const auto center_x = parseDouble(
    monitor->getValue(diagnostic_name.value(), "center_x"));
  const auto center_y = parseDouble(
    monitor->getValue(diagnostic_name.value(), "center_y"));

  if (!current_x || !current_y || !center_x || !center_y)
  {
    return BT::NodeStatus::FAILURE;
  }

  double force = 40.0;
  auto input_force = getInput<double>("force");
  if (input_force)
  {
    force = std::abs(input_force.value());
  }

  double target_x = center_x.value();
  double target_y = center_y.value();

  const auto shape = monitor->getValue(diagnostic_name.value(), "shape");
  if (shape == "rectangle" || shape == "rect")
  {
    const auto width = parseDouble(
      monitor->getValue(diagnostic_name.value(), "width"));
    const auto height = parseDouble(
      monitor->getValue(diagnostic_name.value(), "height"));

    if (width && height)
    {
      const double half_width = width.value() * 0.5;
      const double half_height = height.value() * 0.5;

      target_x = std::clamp(
        current_x.value(),
        center_x.value() - half_width,
        center_x.value() + half_width);
      target_y = std::clamp(
        current_y.value(),
        center_y.value() - half_height,
        center_y.value() + half_height);
    }
  }

  const double world_x = target_x - current_x.value();
  const double world_y = target_y - current_y.value();
  const double norm = std::hypot(world_x, world_y);

  if (norm <= 1e-6)
  {
    setOutput("output_x", 0.0);
    setOutput("output_y", 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  Navigator::SharedPtr navigator_msg;
  {
    std::lock_guard<std::mutex> lock(navigator_mutex_);
    navigator_msg = last_navigator_msg_;
  }

  if (!navigator_msg)
  {
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = yawFromQuaternion(navigator_msg->position.orientation);
  const double unit_world_x = world_x / norm;
  const double unit_world_y = world_y / norm;

  const double body_force_x =
    force * (std::cos(yaw) * unit_world_x + std::sin(yaw) * unit_world_y);
  const double body_force_y =
    force * (-std::sin(yaw) * unit_world_x + std::cos(yaw) * unit_world_y);

  setOutput("output_x", body_force_x);
  setOutput("output_y", body_force_y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_bt
