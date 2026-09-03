#pragma once

#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "sura_msgs/srv/controller_interlock.hpp"

namespace sura_bt
{

class ControllerSwitchBase
{
protected:
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using SetHardwareComponentState =
    controller_manager_msgs::srv::SetHardwareComponentState;
  using ControllerInterlock = sura_msgs::srv::ControllerInterlock;

  static rclcpp::Node::SharedPtr getRosNode(
    const BT::NodeConfiguration & config);

  static std::vector<std::string> getSwitchableControllers(
    const BT::NodeConfiguration & config);

  static std::string switchControllerServiceName(
    const BT::NodeConfiguration & config);

  static std::string hardwareComponentServiceName(
    const BT::NodeConfiguration & config);

  static std::string controllerInterlockServiceName(
    const BT::NodeConfiguration & config);
};

class ActivateControllers : public BT::SyncActionNode, protected ControllerSwitchBase
{
public:
  ActivateControllers(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
};

class DeactivateControllers : public BT::SyncActionNode, protected ControllerSwitchBase
{
public:
  DeactivateControllers(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  bool has_last_request_time_{false};
  rclcpp::Time last_request_time_;
};

class DeactivateSystem : public BT::SyncActionNode, protected ControllerSwitchBase
{
public:
  DeactivateSystem(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<SetHardwareComponentState>::SharedPtr hardware_client_;
  std::map<std::string, rclcpp::Time> last_request_time_;
};

class SetControllerInterlock : public BT::SyncActionNode, protected ControllerSwitchBase
{
public:
  SetControllerInterlock(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<ControllerInterlock>::SharedPtr interlock_client_;
  bool has_last_request_{false};
  rclcpp::Time last_request_time_;
  bool last_enabled_{false};
  std::string last_reason_;
  std::string last_controllers_;
};

}  // namespace sura_bt
