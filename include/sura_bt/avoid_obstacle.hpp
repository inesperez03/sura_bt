#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/sura_velocity_command.hpp"

namespace sura_safety {class DiagnosticsMonitor;}

namespace sura_bt
{

class AvoidObstacle : public BT::StatefulActionNode
{
public:
  using Mode = sura_actions::srv::SetControlMode;
  using VelocityCommand = sura_msgs::msg::SuraVelocityCommand;

  AvoidObstacle(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  static const char * main_description();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus finish(BT::NodeStatus status, const std::string & message);
  void publishVelocity(double lateral_velocity);
  void stopMotion();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<sura_safety::DiagnosticsMonitor> monitor_;
  rclcpp::Client<Mode>::SharedPtr mode_client_;
  rclcpp::Client<Mode>::SharedFuture mode_future_;
  rclcpp::Publisher<VelocityCommand>::SharedPtr velocity_pub_;
  std::string robot_namespace_;
  std::string requester_;
  double lateral_velocity_{0.0};
  bool mode_requested_{false};
  bool mode_ready_{false};
  bool command_started_{false};
  std::chrono::steady_clock::time_point started_;
};

}  // namespace sura_bt
