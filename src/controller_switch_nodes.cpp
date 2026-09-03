#include "sura_bt/controller_switch_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <map>
#include <set>
#include <cmath>
#include <sstream>

#include "lifecycle_msgs/msg/state.hpp"

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

std::string trim(std::string value)
{
  const auto not_space = [](unsigned char c) {
    return !std::isspace(c);
  };

  value.erase(
    value.begin(),
    std::find_if(value.begin(), value.end(), not_space));
  value.erase(
    std::find_if(value.rbegin(), value.rend(), not_space).base(),
    value.end());
  return value;
}

std::vector<std::string> splitControllers(const std::string & text)
{
  std::vector<std::string> controllers;
  std::stringstream stream(text);
  std::string item;

  while (std::getline(stream, item, ','))
  {
    item = trim(item);
    if (!item.empty())
    {
      controllers.push_back(item);
    }
  }

  return controllers;
}

bool isAllControllersRequest(const std::vector<std::string> & controllers)
{
  return controllers.size() == 1 && controllers.front() == "all";
}

std::vector<std::string> deduplicatePreservingOrder(const std::vector<std::string> & values)
{
  std::vector<std::string> result;
  std::set<std::string> seen;

  for (const auto & value : values)
  {
    if (seen.insert(value).second)
    {
      result.push_back(value);
    }
  }

  return result;
}

std::vector<std::string> stackForController(const std::string & controller)
{
  static const std::map<std::string, std::vector<std::string>> stacks{
    {"body_force", {"body_force"}},
    {"depth_hold", {"body_force", "depth_hold"}},
    {"body_velocity", {"body_force", "depth_hold", "body_velocity"}},
    {"stabilize", {"body_force", "stabilize"}},
    {"position_hold", {"body_force", "depth_hold", "body_velocity", "position_hold"}},
    {"mpc_4dof", {"body_force", "mpc_4dof"}},
    {"thruster_test_controller", {"thruster_test_controller"}},
    {"lights_controller", {"lights_controller"}}
  };

  const auto stack = stacks.find(controller);
  if (stack != stacks.end())
  {
    return stack->second;
  }

  return {controller};
}

std::vector<std::string> expandControllerStack(const std::vector<std::string> & controllers)
{
  std::vector<std::string> expanded;

  for (const auto & controller : controllers)
  {
    const auto stack = stackForController(controller);
    expanded.insert(expanded.end(), stack.begin(), stack.end());
  }

  return deduplicatePreservingOrder(expanded);
}

std::vector<std::string> orderLikeSwitchableControllers(
  const std::vector<std::string> & controllers,
  const std::vector<std::string> & switchable_controllers)
{
  std::vector<std::string> ordered;
  const std::set<std::string> requested(controllers.begin(), controllers.end());
  std::set<std::string> added;

  for (const auto & controller : switchable_controllers)
  {
    if (requested.count(controller) > 0 && added.insert(controller).second)
    {
      ordered.push_back(controller);
    }
  }

  for (const auto & controller : controllers)
  {
    if (added.insert(controller).second)
    {
      ordered.push_back(controller);
    }
  }

  return ordered;
}

std::vector<std::string> controllersNotInDesiredStack(
  const std::vector<std::string> & desired_stack,
  const std::vector<std::string> & switchable_controllers)
{
  std::vector<std::string> deactivate_controllers;
  const std::set<std::string> desired(desired_stack.begin(), desired_stack.end());

  for (const auto & controller : switchable_controllers)
  {
    if (desired.count(controller) == 0)
    {
      deactivate_controllers.push_back(controller);
    }
  }

  return deactivate_controllers;
}

std::string joinControllers(const std::vector<std::string> & controllers)
{
  std::stringstream controller_list;
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    if (i > 0)
    {
      controller_list << ",";
    }
    controller_list << controllers[i];
  }

  return controller_list.str();
}

int64_t secondsToNanoseconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0)
  {
    seconds = 0.0;
  }

  return static_cast<int64_t>(seconds * 1000000000.0);
}

void setTimeout(
  controller_manager_msgs::srv::SwitchController::Request & request,
  double timeout_sec)
{
  const auto timeout_ns = secondsToNanoseconds(timeout_sec);
  request.timeout.sec = static_cast<int32_t>(timeout_ns / 1000000000LL);
  request.timeout.nanosec = static_cast<uint32_t>(timeout_ns % 1000000000LL);
}

BT::PortsList activateControllerPorts()
{
  return {
    BT::InputPort<std::string>(
      "controllers",
      "Comma-separated controller names to activate for the requested behavior.")
  };
}

BT::PortsList deactivateControllerPorts()
{
  return {
    BT::InputPort<std::string>(
      "controllers",
      "Comma-separated controller names to deactivate, or 'all' for every switchable controller."),
    BT::InputPort<double>(
      "cooldown_sec",
      "Minimum time between repeated deactivation requests, in seconds.")
  };
}

std::string namespacedSystemName(
  const BT::NodeConfiguration & config,
  const std::string & system)
{
  const auto robot_namespace =
    stripSlashes(config.blackboard->get<std::string>("robot_namespace"));
  const auto normalized_system = stripSlashes(system);

  if (normalized_system.empty())
  {
    return normalized_system;
  }

  if (robot_namespace.empty() ||
    normalized_system.rfind(robot_namespace + "_", 0) == 0)
  {
    return normalized_system;
  }

  return robot_namespace + "_" + normalized_system;
}

BT::NodeStatus callSwitchController(
  const rclcpp::Node::SharedPtr & ros_node,
  const rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr & client,
  const std::vector<std::string> & activate_controllers,
  const std::vector<std::string> & deactivate_controllers,
  bool strict,
  bool activate_asap,
  double timeout_sec)
{
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  if (activate_controllers.empty() && deactivate_controllers.empty())
  {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[sura_bt] SwitchControllers requested with no controllers");
    return BT::NodeStatus::FAILURE;
  }

  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] switch_controller service is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate_controllers;
  request->deactivate_controllers = deactivate_controllers;
  request->strictness = strict ?
    SwitchController::Request::STRICT :
    SwitchController::Request::BEST_EFFORT;
  request->activate_asap = activate_asap;
  setTimeout(*request, timeout_sec);

  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    ros_node,
    future,
    std::chrono::duration<double>(timeout_sec));

  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] switch_controller request timed out");
    return BT::NodeStatus::FAILURE;
  }

  if (!future.get()->ok)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] switch_controller request failed");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

std::vector<std::string> resolveControllers(
  const BT::NodeConfiguration & config,
  const std::string & controllers_text,
  const rclcpp::Node::SharedPtr & ros_node,
  const std::string & node_name)
{
  auto controllers = splitControllers(controllers_text);
  if (isAllControllersRequest(controllers))
  {
    controllers = config.blackboard->get<std::vector<std::string>>(
      "switchable_controllers");
    std::stringstream controller_list;
    for (size_t i = 0; i < controllers.size(); ++i)
    {
      if (i > 0)
      {
        controller_list << ",";
      }
      controller_list << controllers[i];
    }

    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] %s resolved controllers=\"all\" to: %s",
      node_name.c_str(),
      controller_list.str().c_str());
  }

  return controllers;
}

}  // namespace

rclcpp::Node::SharedPtr ControllerSwitchBase::getRosNode(
  const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

std::vector<std::string> ControllerSwitchBase::getSwitchableControllers(
  const BT::NodeConfiguration & config)
{
  return config.blackboard->get<std::vector<std::string>>(
    "switchable_controllers");
}

std::string ControllerSwitchBase::switchControllerServiceName(
  const BT::NodeConfiguration & config)
{
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");
  const auto normalized_namespace = stripSlashes(robot_namespace);

  if (normalized_namespace.empty())
  {
    return "/controller/arbitrator/switch_controller";
  }

  return "/" + normalized_namespace + "/controller/arbitrator/switch_controller";
}

std::string ControllerSwitchBase::hardwareComponentServiceName(
  const BT::NodeConfiguration & config)
{
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");
  const auto normalized_namespace = stripSlashes(robot_namespace);

  if (normalized_namespace.empty())
  {
    return "/controller/controller_manager/set_hardware_component_state";
  }

  return "/" + normalized_namespace +
    "/controller/controller_manager/set_hardware_component_state";
}

std::string ControllerSwitchBase::controllerInterlockServiceName(
  const BT::NodeConfiguration & config)
{
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");
  const auto normalized_namespace = stripSlashes(robot_namespace);

  if (normalized_namespace.empty())
  {
    return "/controller/arbitrator/controller_interlock";
  }

  return "/" + normalized_namespace + "/controller/arbitrator/controller_interlock";
}

ActivateControllers::ActivateControllers(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  switch_client_ = getRosNode(config)->create_client<SwitchController>(
    switchControllerServiceName(config));
}

BT::PortsList ActivateControllers::providedPorts()
{
  return activateControllerPorts();
}

const char * ActivateControllers::main_description()
{
  return "Activates the requested set of controllers.";
}

BT::NodeStatus ActivateControllers::tick()
{
  const auto ros_node = getRosNode(config());
  auto controllers_text = getInput<std::string>("controllers");
  if (!controllers_text)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] ActivateControllers requires controllers input");
    return BT::NodeStatus::FAILURE;
  }

  const auto controllers = splitControllers(controllers_text.value());
  if (isAllControllersRequest(controllers))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] ActivateControllers does not support controllers=\"all\"");
    return BT::NodeStatus::FAILURE;
  }

  const auto switchable_controllers = getSwitchableControllers(config());
  const auto desired_stack = orderLikeSwitchableControllers(
    expandControllerStack(controllers),
    switchable_controllers);
  const auto deactivate_controllers = controllersNotInDesiredStack(
    desired_stack,
    switchable_controllers);

  RCLCPP_INFO(
    ros_node->get_logger(),
    "[sura_bt] ActivateControllers resolved requested=\"%s\" to activate=\"%s\" deactivate=\"%s\"",
    controllers_text.value().c_str(),
    joinControllers(desired_stack).c_str(),
    joinControllers(deactivate_controllers).c_str());

  const auto status = callSwitchController(
    ros_node,
    switch_client_,
    desired_stack,
    deactivate_controllers,
    false,
    true,
    2.0);

  if (status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] Activated controllers: %s",
      controllers_text.value().c_str());
  }

  return status;
}

DeactivateControllers::DeactivateControllers(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  switch_client_ = getRosNode(config)->create_client<SwitchController>(
    switchControllerServiceName(config));
}

BT::PortsList DeactivateControllers::providedPorts()
{
  return deactivateControllerPorts();
}

const char * DeactivateControllers::main_description()
{
  return "Deactivates the requested set of controllers.";
}

BT::NodeStatus DeactivateControllers::tick()
{
  const auto ros_node = getRosNode(config());
  auto controllers_text = getInput<std::string>("controllers");
  if (!controllers_text)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] DeactivateControllers requires controllers input");
    return BT::NodeStatus::FAILURE;
  }

  const auto controllers = resolveControllers(
    config(),
    controllers_text.value(),
    ros_node,
    "DeactivateControllers");

  auto cooldown_sec = getInput<double>("cooldown_sec");
  const double cooldown = cooldown_sec && cooldown_sec.value() > 0.0 ?
    cooldown_sec.value() : 0.0;

  const auto now = ros_node->now();
  if (cooldown > 0.0 && has_last_request_time_ &&
    (now - last_request_time_).seconds() < cooldown)
  {
    RCLCPP_DEBUG(
      ros_node->get_logger(),
      "[sura_bt] Skipping DeactivateControllers: cooldown %.3fs",
      cooldown);
    return BT::NodeStatus::SUCCESS;
  }

  const auto status = callSwitchController(
    ros_node,
    switch_client_,
    {},
    controllers,
    false,
    true,
    2.0);

  if (status == BT::NodeStatus::SUCCESS)
  {
    last_request_time_ = now;
    has_last_request_time_ = true;

    RCLCPP_INFO(
      ros_node->get_logger(),
      "[sura_bt] Deactivated controllers: %s",
      controllers_text.value().c_str());
  }

  return status;
}

DeactivateSystem::DeactivateSystem(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  hardware_client_ = getRosNode(config)->create_client<SetHardwareComponentState>(
    hardwareComponentServiceName(config));
}

BT::PortsList DeactivateSystem::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "system",
      "Hardware system or component to deactivate."),
    BT::InputPort<double>(
      "cooldown_sec",
      "Minimum time between repeated deactivation requests for the same system, in seconds.")
  };
}

const char * DeactivateSystem::main_description()
{
  return "Deactivates a requested hardware system or component.";
}

BT::NodeStatus DeactivateSystem::tick()
{
  const auto ros_node = getRosNode(config());
  const double timeout = 5.0;
  auto system = getInput<std::string>("system");
  auto cooldown_sec = getInput<double>("cooldown_sec");
  const double cooldown = cooldown_sec && cooldown_sec.value() > 0.0 ?
    cooldown_sec.value() : 5.0;
  if (!system)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] DeactivateSystem requires system input");
    return BT::NodeStatus::FAILURE;
  }

  const std::string component_name = namespacedSystemName(config(), system.value());
  if (component_name.empty())
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] DeactivateSystem received empty system input");
    return BT::NodeStatus::FAILURE;
  }

  const auto now = ros_node->now();
  const auto last_request = last_request_time_.find(component_name);
  if (last_request != last_request_time_.end() &&
    (now - last_request->second).seconds() < cooldown)
  {
    RCLCPP_DEBUG(
      ros_node->get_logger(),
      "[sura_bt] Skipping DeactivateSystem for '%s': cooldown %.3fs",
      component_name.c_str(),
      cooldown);
    return BT::NodeStatus::SUCCESS;
  }

  last_request_time_[component_name] = now;

  if (!hardware_client_->wait_for_service(std::chrono::duration<double>(timeout)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] set_hardware_component_state service is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<SetHardwareComponentState::Request>();
  request->name = component_name;
  request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  request->target_state.label = "inactive";

  auto future = hardware_client_->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    ros_node,
    future,
    std::chrono::duration<double>(timeout));

  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] set_hardware_component_state request timed out");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response->ok)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] Failed to deactivate hardware component '%s'. current_state=%s",
      component_name.c_str(),
      response->state.label.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(
    ros_node->get_logger(),
    "[sura_bt] System deactivated: %s",
    component_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

SetControllerInterlock::SetControllerInterlock(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  interlock_client_ = getRosNode(config)->create_client<ControllerInterlock>(
    controllerInterlockServiceName(config));
}

BT::PortsList SetControllerInterlock::providedPorts()
{
  return {
    BT::InputPort<bool>(
      "enabled",
      "Whether the controller interlock should be enabled."),
    BT::InputPort<std::string>(
      "reason",
      "Mission-level reason for changing the controller interlock state."),
    BT::InputPort<std::string>(
      "controllers",
      "Comma-separated controller names affected when the interlock is enabled.")
  };
}

const char * SetControllerInterlock::main_description()
{
  return "Enables or disables a controller interlock for selected controllers.";
}

BT::NodeStatus SetControllerInterlock::tick()
{
  const auto ros_node = getRosNode(config());
  auto enabled = getInput<bool>("enabled");
  auto reason = getInput<std::string>("reason");
  auto controllers_text = getInput<std::string>("controllers");

  if (!enabled || !reason)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] SetControllerInterlock requires enabled and reason inputs");
    return BT::NodeStatus::FAILURE;
  }

  if (enabled.value() && !controllers_text)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] SetControllerInterlock requires controllers input when enabled=true");
    return BT::NodeStatus::FAILURE;
  }

  const std::string controllers_value = controllers_text ? controllers_text.value() : "";
  const auto now = ros_node->now();
  constexpr double repeat_cooldown_sec = 1.0;
  if (has_last_request_ &&
    last_enabled_ == enabled.value() &&
    last_reason_ == reason.value() &&
    last_controllers_ == controllers_value &&
    (now - last_request_time_).seconds() < repeat_cooldown_sec)
  {
    return BT::NodeStatus::SUCCESS;
  }

  constexpr double timeout = 2.0;
  if (!interlock_client_->wait_for_service(std::chrono::duration<double>(timeout)))
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] controller_interlock service is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<ControllerInterlock::Request>();
  request->command = ControllerInterlock::Request::SET;
  request->enabled = enabled.value();
  request->reason = reason.value();
  if (controllers_text)
  {
    request->blocked_controllers = splitControllers(controllers_text.value());
  }

  auto future = interlock_client_->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    ros_node,
    future,
    std::chrono::duration<double>(timeout));

  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] controller_interlock request timed out");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] controller_interlock request failed: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (enabled.value())
  {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[sura_bt] Controller interlock enabled. reason=%s controllers=%s",
      reason.value().c_str(),
      controllers_value.c_str());
  }
  has_last_request_ = true;
  last_request_time_ = now;
  last_enabled_ = enabled.value();
  last_reason_ = reason.value();
  last_controllers_ = controllers_value;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_bt
