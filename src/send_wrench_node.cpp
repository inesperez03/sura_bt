#include "sura_bt/send_wrench_node.hpp"

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
    BT::InputPort<std::string>("reason"),
    BT::InputPort<int>("priority"),
    BT::InputPort<double>("force_x"),
    BT::InputPort<double>("force_y"),
    BT::InputPort<double>("force_z")
  };
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

  auto force_y = getInput<double>("force_y");
  if (!force_y)
  {
    force_y = 0.0;
  }

  auto force_z = getInput<double>("force_z");
  if (!force_z)
  {
    force_z = 0.0;
  }

  WrenchCommand msg;
  msg.header.stamp = ros_node->now();
  msg.requester = "sura_safety";
  msg.controller = "body_force";
  msg.priority = static_cast<uint8_t>(std::clamp(priority.value(), 1, 100));
  msg.wrench.force.x = force_x.value();
  msg.wrench.force.y = force_y.value();
  msg.wrench.force.z = force_z.value();

  wrench_pub_->publish(msg);

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_bt] Action: send wrench. reason=%s controller=%s priority=%u force=(%.3f, %.3f, %.3f)",
    reason.value().c_str(),
    msg.controller.c_str(),
    msg.priority,
    msg.wrench.force.x,
    msg.wrench.force.y,
    msg.wrench.force.z);

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
    BT::InputPort<double>("force"),
    BT::InputPort<std::string>("diagnostic_name"),
    BT::InputPort<std::string>("diagnostic_suffix"),
    BT::OutputPort<double>("output_x"),
    BT::OutputPort<double>("output_y")
  };
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
