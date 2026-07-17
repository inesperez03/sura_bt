#pragma once

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "sura_msgs/msg/navigator.hpp"
#include "sura_msgs/msg/sura_wrench_command.hpp"

namespace sura_bt
{

class SendWrench : public BT::SyncActionNode
{
public:
  SendWrench(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  using WrenchCommand = sura_msgs::msg::SuraWrenchCommand;

  rclcpp::Publisher<WrenchCommand>::SharedPtr wrench_pub_;
};

class ComputeAreaRecoveryForce : public BT::SyncActionNode
{
public:
  ComputeAreaRecoveryForce(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  using Navigator = sura_msgs::msg::Navigator;

  void navigatorCallback(const Navigator::SharedPtr msg);

  rclcpp::Subscription<Navigator>::SharedPtr navigator_sub_;

  mutable std::mutex navigator_mutex_;
  Navigator::SharedPtr last_navigator_msg_;
};

}  // namespace sura_bt
