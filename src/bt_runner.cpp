#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "yaml-cpp/yaml.h"

#include "sura_bt/register_nodes.hpp"
#include "sura_safety/safety_blackboard.hpp"

static std::string getArgumentValue(
  int argc,
  char ** argv,
  const std::string & key,
  const std::string & default_value)
{
  const std::string prefix = key + ":=";

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);

    if (arg.rfind(prefix, 0) == 0)
    {
      return arg.substr(prefix.size());
    }
  }

  return default_value;
}

static bool isBroadcasterController(
  const std::string & name,
  const std::string & type)
{
  const std::string broadcaster_suffix = "_broadcaster";

  if (name.size() >= broadcaster_suffix.size() &&
    name.compare(
      name.size() - broadcaster_suffix.size(),
      broadcaster_suffix.size(),
      broadcaster_suffix) == 0)
  {
    return true;
  }

  return type.find("Broadcaster") != std::string::npos;
}

static std::vector<std::string> loadSwitchableControllers(
  const std::string & robot_namespace)
{
  const std::string description_package = robot_namespace + "_description";
  const std::string description_share =
    ament_index_cpp::get_package_share_directory(description_package);

  const std::string bringup_description_file =
    description_share + "/config/bringup_description.yaml";
  const auto bringup = YAML::LoadFile(bringup_description_file);
  const auto ros2_control = bringup["ros2_control"];

  if (!ros2_control)
  {
    throw std::runtime_error(
      "Missing ros2_control section in " + bringup_description_file);
  }

  const auto params_package_node = ros2_control["params_package"];
  const auto params_file_node = ros2_control["params"];
  if (!params_package_node || !params_file_node)
  {
    throw std::runtime_error(
      "Missing ros2_control.params_package or ros2_control.params in " +
      bringup_description_file);
  }

  const std::string params_package = params_package_node.as<std::string>();
  const std::string params_file = params_file_node.as<std::string>();
  const std::string params_share =
    ament_index_cpp::get_package_share_directory(params_package);
  const std::string ros2_control_params_file =
    params_share + "/" + params_file;

  const auto params = YAML::LoadFile(ros2_control_params_file);
  const auto normalized_namespace =
    robot_namespace.empty() ? std::string{} : "/" + robot_namespace;
  const auto controller_manager_key =
    normalized_namespace + "/controller/controller_manager";

  YAML::Node controller_manager = params[controller_manager_key];
  if (!controller_manager)
  {
    for (const auto & entry : params)
    {
      const auto key = entry.first.as<std::string>();
      if (key.find("/controller/controller_manager") != std::string::npos)
      {
        controller_manager = entry.second;
        break;
      }
    }
  }

  if (!controller_manager || !controller_manager["ros__parameters"])
  {
    throw std::runtime_error(
      "Could not find controller_manager ros__parameters in " +
      ros2_control_params_file);
  }

  const auto ros_parameters = controller_manager["ros__parameters"];
  std::vector<std::string> controllers;
  for (const auto & entry : ros_parameters)
  {
    const auto name = entry.first.as<std::string>();
    const auto config = entry.second;
    if (!config.IsMap() || !config["type"])
    {
      continue;
    }

    const auto type = config["type"].as<std::string>();
    if (isBroadcasterController(name, type))
    {
      continue;
    }

    controllers.push_back(name);
  }

  return controllers;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<rclcpp::Node>("sura_bt_runner");

  const std::string robot_namespace =
    getArgumentValue(argc, argv, "robot_namespace", "bluerov");

  RCLCPP_INFO(
    ros_node->get_logger(),
    "robot_namespace='%s'",
    robot_namespace.c_str());

  BT::BehaviorTreeFactory factory;

  const std::string tree_file =
    ament_index_cpp::get_package_share_directory("sura_bt") +
    "/trees/mission.xml";
  const std::string safety_tree_file =
    ament_index_cpp::get_package_share_directory("sura_safety") +
    "/trees/safety_branch.xml";
  const std::string autonomous_tree_file =
    ament_index_cpp::get_package_share_directory("sura_bt") +
    "/trees/autonomous_branch.xml";
  const std::string teleop_tree_file =
    ament_index_cpp::get_package_share_directory("sura_bt") +
    "/trees/teleop_branch.xml";

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading tree: %s",
    tree_file.c_str()
  );
  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading safety tree: %s",
    safety_tree_file.c_str()
  );
  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading autonomous tree: %s",
    autonomous_tree_file.c_str()
  );
  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading teleop tree: %s",
    teleop_tree_file.c_str()
  );

  auto blackboard = BT::Blackboard::create();
  const auto switchable_controllers =
    loadSwitchableControllers(robot_namespace);

  blackboard->set("switchable_controllers", switchable_controllers);

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loaded %zu switchable controllers from ros2_control params",
    switchable_controllers.size());

  sura_safety::configureSafety(
    ros_node,
    factory,
    blackboard,
    robot_namespace);

  sura_bt::registerNodes(factory);

  factory.registerBehaviorTreeFromFile(safety_tree_file);
  factory.registerBehaviorTreeFromFile(autonomous_tree_file);
  factory.registerBehaviorTreeFromFile(teleop_tree_file);
  factory.registerBehaviorTreeFromFile(tree_file);

  auto tree = factory.createTree("Mission", blackboard);

  BT::PublisherZMQ publisher_zmq(tree);

  rclcpp::Rate rate(10.0);

  while (rclcpp::ok())
  {
    tree.tickRoot();

    rclcpp::spin_some(ros_node);

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
