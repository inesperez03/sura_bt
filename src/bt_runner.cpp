#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "sura_safety/diagnostics_monitor.hpp"

static std::string toUpper(std::string text)
{
  std::transform(
    text.begin(),
    text.end(),
    text.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); }
  );

  return text;
}

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<rclcpp::Node>("sura_bt_runner");

  const std::string robot_namespace =
    getArgumentValue(argc, argv, "robot_namespace", "bluerov");

  const std::string diagnostic_prefix =
    "/" + toUpper(robot_namespace);

  RCLCPP_INFO(
    ros_node->get_logger(),
    "robot_namespace='%s', diagnostic_prefix='%s'",
    robot_namespace.c_str(),
    diagnostic_prefix.c_str()
  );

  auto diagnostics_monitor =
    std::make_shared<sura_safety::DiagnosticsMonitor>(
      ros_node,
      "/diagnostics_agg");

  BT::BehaviorTreeFactory factory;

  const std::string safety_plugin_path =
    ament_index_cpp::get_package_prefix("sura_safety") +
    "/lib/libsura_safety.so";

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading safety plugin: %s",
    safety_plugin_path.c_str()
  );

  factory.registerFromPlugin(safety_plugin_path);

  const std::string tree_file =
    ament_index_cpp::get_package_share_directory("sura_bt") +
    "/trees/mission.xml";

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading tree: %s",
    tree_file.c_str()
  );

  auto blackboard = BT::Blackboard::create();

  blackboard->set("ros_node", ros_node);
  blackboard->set("robot_namespace", robot_namespace);
  blackboard->set("diagnostic_prefix", diagnostic_prefix);
  blackboard->set("diagnostics_monitor", diagnostics_monitor);

  auto tree = factory.createTreeFromFile(tree_file, blackboard);

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
