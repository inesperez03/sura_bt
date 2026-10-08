#include <algorithm>
#include <exception>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <thread>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cstdarg>
#include <cstdio>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "yaml-cpp/yaml.h"
#include "nlohmann/json.hpp"
#include "tinyxml2.h"

#include "sura_bt/fleet_tree.hpp"
#include "sura_bt/mission_failure_reasons.hpp"
#include "sura_bt/safety_ask_coordinator.hpp"
#include "sura_bt/register_nodes.hpp"
#include "sura_safety/safety_blackboard.hpp"
#include "sura_safety/diagnostics_monitor.hpp"
#include "sura_safety/safety_ask_state.hpp"
#include "sura_safety/critical_safety_state.hpp"

namespace
{
std::mutex mission_error_log_mutex;
std::string latest_mission_error_log;
std::chrono::steady_clock::time_point latest_mission_error_time;
rcutils_logging_output_handler_t previous_logging_handler = nullptr;

void missionLoggingOutputHandler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (previous_logging_handler) {
    previous_logging_handler(location, severity, name, timestamp, format, args);
  }
  if (severity < RCUTILS_LOG_SEVERITY_ERROR || !format || !args) {return;}

  char buffer[4096]{};
  va_list copy;
  va_copy(copy, *args);
  const int length = std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  if (length <= 0) {return;}

  std::lock_guard<std::mutex> lock(mission_error_log_mutex);
  latest_mission_error_log.assign(buffer, std::min<std::size_t>(length, sizeof(buffer) - 1));
  latest_mission_error_time = std::chrono::steady_clock::now();
}

void clearMissionErrorLog()
{
  std::lock_guard<std::mutex> lock(mission_error_log_mutex);
  latest_mission_error_log.clear();
}

std::string takeRecentMissionErrorLog()
{
  std::lock_guard<std::mutex> lock(mission_error_log_mutex);
  if (latest_mission_error_log.empty() ||
    std::chrono::steady_clock::now() - latest_mission_error_time > std::chrono::seconds(5))
  {
    latest_mission_error_log.clear();
    return {};
  }
  return latest_mission_error_log;
}
}  // namespace

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

static std::string missionFailureMessage(const nlohmann::json & failure)
{
  std::string action = failure.value("node_type", failure.value("action", "mission action"));
  std::string label;
  for (std::size_t i = 0; i < action.size(); ++i) {
    const auto character = static_cast<unsigned char>(action[i]);
    if (character == '_' || character == '-') {
      if (!label.empty() && label.back() != ' ') {label.push_back(' ');}
      continue;
    }
    if (std::isupper(character) && !label.empty() && label.back() != ' ' &&
      std::islower(static_cast<unsigned char>(label.back())))
    {
      label.push_back(' ');
    }
    label.push_back(static_cast<char>(std::tolower(character)));
  }
  while (!label.empty() && label.back() == ' ') {label.pop_back();}
  constexpr std::string_view action_suffix = " action";
  if (label.size() >= action_suffix.size() &&
    label.compare(label.size() - action_suffix.size(), action_suffix.size(), action_suffix) == 0)
  {
    label.erase(label.size() - action_suffix.size());
  }
  if (label.empty()) {label = "mission";}

  auto reason = failure.value("reason", std::string{});
  if (reason.empty()) {reason = failure.value("terminal_error", std::string{});}
  std::string message = "The mission stopped because the " + label + " step could not finish";
  if (!reason.empty()) {
    message += ": " + reason;
    if (reason.back() != '.' && reason.back() != '!' && reason.back() != '?') {
      message += ".";
    }
  } else {
    message += ".";
  }
  return message;
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


static nlohmann::ordered_json loadRobotProfile(const std::string & robot_namespace)
{
  const std::string package = robot_namespace + "_description";
  const auto share = std::filesystem::path(
    ament_index_cpp::get_package_share_directory(package));
  const auto bringup_path = share / "config" / "bringup_description.yaml";
  const auto bringup = YAML::LoadFile(bringup_path.string());
  const auto description = bringup["description"];
  const auto ros2_control = bringup["ros2_control"];
  if (!description || !description["xacro"] || !ros2_control || !ros2_control["params"]) {
    throw std::runtime_error(
      "bringup_description.yaml must define description.xacro and ros2_control.params for " +
      robot_namespace);
  }

  const std::string description_package = description["package"] ?
    description["package"].as<std::string>() : package;
  const std::string xacro = description["xacro"].as<std::string>();
  const auto xacro_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory(description_package)) / xacro;
  std::string family;
  tinyxml2::XMLDocument xacro_doc;
  if (xacro_doc.LoadFile(xacro_path.string().c_str()) == tinyxml2::XML_SUCCESS) {
    const auto * robot_element = xacro_doc.FirstChildElement("robot");
    if (robot_element && robot_element->Attribute("family")) {
      family = robot_element->Attribute("family");
    }
  }
  if (bringup["robot"] && bringup["robot"]["family"]) {
    family = bringup["robot"]["family"].as<std::string>();
  }

  const std::string params_package = ros2_control["params_package"] ?
    ros2_control["params_package"].as<std::string>() : description_package;
  const std::string params_file = ros2_control["params"].as<std::string>();
  const auto params_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory(params_package)) / params_file;
  const auto params = YAML::LoadFile(params_path.string());
  const auto controller_manager_key = "/" + robot_namespace + "/controller/controller_manager";
  auto controller_manager = params[controller_manager_key]["ros__parameters"];
  if (!controller_manager) {
    for (const auto & entry : params) {
      if (entry.first.as<std::string>().find("/controller/controller_manager") !=
        std::string::npos)
      {
        controller_manager = entry.second["ros__parameters"];
        break;
      }
    }
  }
  nlohmann::ordered_json controllers = nlohmann::ordered_json::array();
  nlohmann::ordered_json sensor_topics = nlohmann::ordered_json::object();
  if (controller_manager) {
    for (const auto & item : controller_manager) {
      if (!item.second.IsMap() || !item.second["type"]) {continue;}
      const auto name = item.first.as<std::string>();
      const auto type = item.second["type"].as<std::string>();
      if (!isBroadcasterController(name, type)) {
        controllers.push_back(name);
      }
    }
  }

  // Keep the sensor list from robot_description; associate it with the active
  // broadcaster configuration, which may expose several outputs per sensor.
  for (const auto & entry : params) {
    const auto node_name = entry.first.as<std::string>();
    if (node_name.rfind("/" + robot_namespace + "/controller/", 0) != 0 ||
      !entry.second.IsMap()) {continue;}
    const auto values = entry.second["ros__parameters"];
    if (!values || !values.IsMap() || !values["sensor_name"] ||
      !values["sensor_name"].IsScalar()) {continue;}
    const auto sensor = values["sensor_name"].as<std::string>();
    if (!sensor_topics.contains(sensor)) {
      sensor_topics[sensor] = nlohmann::ordered_json::array();
    }
    for (const auto & parameter : values) {
      const auto key = parameter.first.as<std::string>();
      // yaw_topic_name is an input to the magnetometer broadcaster.
      if (key != "topic_name" && key != "points_topic" && key != "ping_topic" &&
        key != "up_vector_topic" && key != "temperature_topic" &&
        key != "pressure_topic") {continue;}
      if (!parameter.second.IsScalar()) {continue;}
      auto topic = parameter.second.as<std::string>();
      if (topic.empty()) {continue;}
      if (topic.front() != '/') {
        if (topic.rfind("~/", 0) == 0) {
          topic = node_name + "/" + topic.substr(2);
        } else if (key == "topic_name") {
          topic = "/" + robot_namespace + "/controller/" + topic;
        } else {
          topic = "/" + robot_namespace + "/" + topic;
        }
      }
      auto & topics = sensor_topics[sensor];
      if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
        topics.push_back(topic);
      }
    }
  }
  // The standard IMU broadcaster uses its default output name; this workspace
  // declares that output in the robot's bringup YAML.
  const auto imu = bringup["imu"];
  if (imu && imu["raw_imu_topic"] && imu["raw_imu_topic"].IsScalar()) {
    auto topic = imu["raw_imu_topic"].as<std::string>();
    if (!topic.empty()) {
      if (topic.front() != '/') {topic = "/" + robot_namespace + "/" + topic;}
      auto & topics = sensor_topics["imu_sensor"];
      if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
        topics.push_back(topic);
      }
    }
  }

  return {
    {"name", robot_namespace},
    {"sensors", nlohmann::ordered_json::array()},
    {"sensor_topics", sensor_topics},
    {"actuators", nlohmann::ordered_json::array()},
    {"controllers", controllers},
    {"cameras", nlohmann::ordered_json::array()},
    {"family", family.empty() ? nlohmann::ordered_json(nullptr) : nlohmann::ordered_json(family)},
    {"available_axes", nullptr}
  };
}

static std::string normalizeJointName(std::string name)
{
  const auto slash = name.find('/');
  if (slash != std::string::npos) {
    name.erase(0, slash + 1);
  }
  return name;
}

static std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

static std::optional<std::pair<double, double>> forceRangeFromLookup(
  const std::string & configured_path)
{
  std::filesystem::path csv_path(configured_path);
  if (!csv_path.is_absolute() && !std::filesystem::exists(csv_path)) {
    try {
      csv_path = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("sura_hardware_interface")) /
        csv_path;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  std::ifstream csv(csv_path);
  if (!csv) {return std::nullopt;}
  std::string line;
  std::getline(csv, line);  // CSV header
  std::optional<double> min_force;
  std::optional<double> max_force;
  while (std::getline(csv, line)) {
    std::stringstream row(line);
    std::string pwm, force;
    if (!std::getline(row, pwm, ',') || !std::getline(row, force, ',')) {continue;}
    try {
      const double value = std::stod(force);
      min_force = min_force ? std::min(*min_force, value) : value;
      max_force = max_force ? std::max(*max_force, value) : value;
    } catch (const std::exception &) {
    }
  }
  if (!min_force || !max_force) {return std::nullopt;}
  return std::make_pair(*min_force, *max_force);
}

static nlohmann::ordered_json availableAxesFromMatrix(
  const std_msgs::msg::Float64MultiArray & matrix,
  const std::map<std::string, std::pair<double, double>> & thruster_force_ranges,
  double relative_threshold)
{
  if (matrix.layout.dim.size() < 2 || matrix.layout.dim[0].size != 6) {return nullptr;}
  const auto rows = matrix.layout.dim[0].size;
  const auto cols = matrix.layout.dim[1].size;
  if (cols == 0 || matrix.data.size() != rows * cols) {return nullptr;}

  std::vector<std::string> thruster_order;
  std::stringstream label_stream(matrix.layout.dim[1].label);
  std::string thruster_name;
  while (std::getline(label_stream, thruster_name, ',')) {
    thruster_order.push_back(thruster_name);
  }
  if (thruster_order.size() != cols) {return nullptr;}

  std::array<double, 6> authority{};
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      const auto limit = thruster_force_ranges.find(thruster_order[col]);
      if (limit == thruster_force_ranges.end()) {return nullptr;}
      const double coefficient = matrix.data[row * cols + col];
      const auto [low, high] = limit->second;
      const double max_thruster_force = std::max(std::abs(low), std::abs(high));
      authority[row] += std::abs(coefficient) * max_thruster_force;
    }
  }

  const double max_force_authority =
    std::max({authority[0], authority[1], authority[2]});
  const double max_torque_authority =
    std::max({authority[3], authority[4], authority[5]});
  const double force_threshold = std::max(1.0, relative_threshold * max_force_authority);
  const double torque_threshold = std::max(0.1, relative_threshold * max_torque_authority);
  const std::array<const char *, 6> axes{
    "surge", "sway", "heave", "roll", "pitch", "yaw"};
  nlohmann::ordered_json result = nlohmann::ordered_json::array();
  for (std::size_t axis = 0; axis < axes.size(); ++axis) {
    const double threshold = axis < 3 ? force_threshold : torque_threshold;
    if (authority[axis] >= threshold) {
      result.push_back(axes[axis]);
    }
  }
  return result;
}

static void parseRenderedRobotDescription(
  const std::string & xml,
  nlohmann::ordered_json & profile,
  const std_msgs::msg::Float64MultiArray * matrix,
  double relative_axis_threshold)
{
  tinyxml2::XMLDocument document;
  if (document.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {return;}
  const auto * robot = document.FirstChildElement("robot");
  if (!robot) {return;}
  if (robot->Attribute("family")) {
    profile["family"] = robot->Attribute("family");
  }

  std::set<std::string> sensors;
  std::set<std::string> actuators;
  std::map<std::string, std::pair<double, double>> thruster_force_ranges;
  for (auto * control = robot->FirstChildElement("ros2_control"); control;
    control = control->NextSiblingElement("ros2_control"))
  {
    const std::string component = control->Attribute("name") ?
      control->Attribute("name") : "";
    const auto component_lower = lowercase(component);
    const bool is_sensor_system = component_lower.size() >= 8 &&
      component_lower.compare(component_lower.size() - 8, 8, "_sensors") == 0;
    const bool is_thruster_system = component_lower.find("thruster") != std::string::npos;
    if (is_sensor_system) {
      for (auto * sensor = control->FirstChildElement("sensor"); sensor;
        sensor = sensor->NextSiblingElement("sensor"))
      {
        if (sensor->Attribute("name")) {sensors.insert(sensor->Attribute("name"));}
      }
    }
    for (auto * joint = control->FirstChildElement("joint"); joint;
      joint = joint->NextSiblingElement("joint"))
    {
      const char * raw_name = joint->Attribute("name");
      if (!raw_name) {continue;}
      bool commandable = false;
      for (auto * interface = joint->FirstChildElement("command_interface"); interface;
        interface = interface->NextSiblingElement("command_interface"))
      {
        commandable = commandable || interface->Attribute("name") != nullptr;
      }
      if (!commandable) {continue;}

      const std::string joint_name(raw_name);
      const bool is_thruster = is_thruster_system ||
        lowercase(joint_name).find("thruster") != std::string::npos;
      if (!is_thruster) {
        actuators.insert(normalizeJointName(joint_name));
        continue;
      }

      for (auto * param = joint->FirstChildElement("param"); param;
        param = param->NextSiblingElement("param"))
      {
        const char * param_name = param->Attribute("name");
        if (param_name && std::string(param_name) == "lookup_csv" && param->GetText()) {
          const auto range = forceRangeFromLookup(param->GetText());
          if (range) {thruster_force_ranges[joint_name] = *range;}
          break;
        }
      }
    }
  }
  profile["sensors"] = sensors;
  for (const auto & sensor : sensors) {
    if (!profile["sensor_topics"].contains(sensor)) {
      profile["sensor_topics"][sensor] = nlohmann::ordered_json::array();
    }
  }
  profile["actuators"] = actuators;
  profile["available_axes"] = matrix ?
    availableAxesFromMatrix(*matrix, thruster_force_ranges, relative_axis_threshold) :
    nlohmann::ordered_json(nullptr);
}

static nlohmann::ordered_json writeMissionRobotProfiles(
  const std::vector<std::string> & robots,
  const std::map<std::string, std_msgs::msg::Float64MultiArray> & allocation_matrices,
  const std::map<std::string, std_msgs::msg::String> & rendered_descriptions,
  const std::string & output_path,
  double relative_axis_threshold)
{
  // Cameras are configured by the operator in the generated profile. Preserve
  // them when robot descriptions or allocation matrices refresh the file.
  std::map<std::string, nlohmann::ordered_json> configured_cameras;
  {
    std::ifstream existing(output_path);
    if (existing) {
      try {
        const auto previous = nlohmann::ordered_json::parse(existing);
        if (previous.contains("robots") && previous["robots"].is_array()) {
          for (const auto & robot : previous["robots"]) {
            if (robot.is_object() && robot.contains("name") && robot["name"].is_string() &&
              robot.contains("cameras") && robot["cameras"].is_array())
            {
              configured_cameras[robot["name"].get<std::string>()] = robot["cameras"];
            }
          }
        }
      } catch (const nlohmann::ordered_json::exception & exception) {
        throw std::runtime_error(
                "Could not read existing robot profile file " + output_path +
                ": " + exception.what());
      }
    }
  }
  nlohmann::ordered_json profiles = nlohmann::ordered_json::array();
  for (const auto & robot : robots) {
    auto profile = loadRobotProfile(robot);
    const auto cameras = configured_cameras.find(robot);
    if (cameras != configured_cameras.end()) {
      profile["cameras"] = cameras->second;
    }
    const auto description = rendered_descriptions.find(robot);
    const auto matrix = allocation_matrices.find(robot);
    if (description != rendered_descriptions.end()) {
      parseRenderedRobotDescription(
        description->second.data, profile,
        matrix == allocation_matrices.end() ? nullptr : &matrix->second,
        relative_axis_threshold);
    }
    profiles.push_back(std::move(profile));
  }

  const auto path = std::filesystem::path(output_path);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Could not write robot profile file: " + output_path);
  }
  output << nlohmann::ordered_json{{"robots", profiles}}.dump(2) << std::endl;
  return profiles;
}

static std::string getTreeFile(
  const std::string & package_name,
  const std::string & relative_path)
{
  const auto share_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory(package_name));
  const auto installed_file = share_dir / relative_path;
  const auto workspace_file =
    share_dir.parent_path().parent_path().parent_path().parent_path() /
    "src" / package_name / relative_path;

  if (std::filesystem::exists(workspace_file))
  {
    return workspace_file.string();
  }

  return installed_file.string();
}

static std::string normalizedRobotNamespace(const std::string & robot_namespace)
{
  std::string result;
  for (const auto character : robot_namespace)
  {
    if (std::isalnum(static_cast<unsigned char>(character)) || character == '_' || character == '-')
    {
      result += character;
    }
  }

  if (result.empty())
  {
    throw std::runtime_error("robot_namespace must contain at least one letter or digit");
  }

  return result;
}

static std::string getSafetyTreeFile(const std::string & robot_namespace)
{
  const auto relative_path =
    "trees/" + normalizedRobotNamespace(robot_namespace) + "_safety_branch.xml";
  const auto tree_file = getTreeFile("sura_safety", relative_path);

  if (!std::filesystem::exists(tree_file))
  {
    throw std::runtime_error(
      "No safety tree exists for robot_namespace='" + robot_namespace + "'. Expected " +
      relative_path);
  }

  return tree_file;
}

static std::vector<std::string> discoverRobotsFromDiagnostics(
  const rclcpp::Node::SharedPtr & node)
{
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  std::vector<std::string> latest;
  auto subscription = node->create_subscription<DiagnosticArray>(
    "/diagnostics_agg", rclcpp::QoS(10),
    [&](const DiagnosticArray::SharedPtr msg) {
      latest = sura_bt::robotsFromDiagnostics(*msg);
    });

  std::vector<std::string> stable;
  auto changed_at = std::chrono::steady_clock::now();
  auto next_log = changed_at;
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    const auto now = std::chrono::steady_clock::now();
    if (latest != stable)
    {
      stable = latest;
      changed_at = now;
    }
    if (!stable.empty() && now - changed_at >= std::chrono::seconds(3))
    {
      return stable;
    }
    if (now >= next_log)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Waiting for a stable robot list from /diagnostics_agg (%zu seen)",
        stable.size());
      next_log = now + std::chrono::seconds(5);
    }
    rate.sleep();
  }
  throw std::runtime_error("Stopped while waiting for robots in /diagnostics_agg");
}

static std::string joinRobotNamespaces(const std::vector<std::string> & robots)
{
  std::string joined;
  for (const auto & robot : robots)
  {
    if (!joined.empty()) {joined += ',';}
    joined += robot;
  }
  return joined;
}

class MissionProgress
{
public:
  MissionProgress(
    const rclcpp::Node::SharedPtr & node, const BT::Blackboard::Ptr & blackboard,
    const std::shared_ptr<sura_bt::MissionFailureReasons> & failure_reasons,
    const std::shared_ptr<sura_safety::CriticalSafetyState> & critical_safety_state)
  : blackboard_(blackboard), node_(node), failure_reasons_(failure_reasons),
    critical_safety_state_(critical_safety_state),
    session_id_(std::to_string(std::chrono::system_clock::now().time_since_epoch().count()))
  {
    publisher_ = node->create_publisher<std_msgs::msg::String>(
      "~/mission_status", rclcpp::QoS(1).reliable().transient_local());
    publish("idle");
  }

  void attach(BT::Tree & tree, const std::string & autonomous_file)
  {
    subscribers_.clear();
    action_names_.clear();
    tinyxml2::XMLDocument xml;
    if (xml.LoadFile(autonomous_file.c_str()) == tinyxml2::XML_SUCCESS) {
      std::function<void(const tinyxml2::XMLElement *)> visit =
        [&](const tinyxml2::XMLElement * element) {
          if (const char * name = element->Attribute("name")) {action_names_.insert(name);}
          for (auto * child = element->FirstChildElement(); child;
            child = child->NextSiblingElement()) {visit(child);}
        };
      if (auto * root = xml.RootElement()) {visit(root);}
    }
    for (const auto & node : tree.nodes) {
      subscribers_.push_back(node->subscribeToStatusChange(
        [this](BT::TimePoint, const BT::TreeNode & changed,
          BT::NodeStatus, BT::NodeStatus status) {
          if (changed.name() == "mission_running" && status == BT::NodeStatus::SUCCESS &&
            !run_open_) {
            ++run_number_;
            run_open_ = true;
            active_.clear();
            completed_.clear();
            completed_order_.clear();
            failed_.clear();
            failure_ = nullptr;
            safety_ = nullptr;
            failure_reasons_->clear();
            event_ = {{"type", "mission_started"}};
            events_.push_back(event_);
            dirty_ = true;
          }
          if (changed.registrationName() == "MissionControl" &&
            status == BT::NodeStatus::FAILURE) {control_failed_ = true;}
          if (changed.type() != BT::NodeType::ACTION ||
            action_names_.count(changed.name()) == 0 ||
            changed.registrationName() == "VariableSet" ||
            changed.registrationName() == "MissionCompleted") {return;}
          const auto name = changed.name();
          if (status == BT::NodeStatus::RUNNING && active_.insert(name).second) {
            clearMissionErrorLog();
            event_ = { {"type", "action_started"}, {"action", name},
              {"node_type", changed.registrationName()} };
            events_.push_back(event_);
            dirty_ = true;
          } else if (status == BT::NodeStatus::SUCCESS &&
            completed_.insert(name).second) {
            active_.erase(name);
            completed_order_.push_back(name);
            event_ = { {"type", "action_completed"}, {"action", name},
              {"node_type", changed.registrationName()} };
            events_.push_back(event_);
            dirty_ = true;
          } else if (status == BT::NodeStatus::FAILURE) {
            active_.erase(name);
            failed_.insert(name);
            event_ = { {"type", "action_failed"}, {"action", name},
              {"node_type", changed.registrationName()} };
            const auto reason = failure_reasons_->take(name);
            if (!reason.empty()) {event_["reason"] = reason;}
            const auto terminal_error = takeRecentMissionErrorLog();
            if (!terminal_error.empty()) {event_["terminal_error"] = terminal_error;}
            failure_ = event_;
          }
        }));
    }
  }

  void afterTick(bool safety_pending)
  {
    std::string raw;
    blackboard_->get("mission_state", raw);
    const std::string state = safety_pending ? "paused" : raw;
    if (state == "aborted")
    {
      const auto critical = critical_safety_state_->first();
      if (critical)
      {
        safety_ = {{"diagnostic", critical->first}, {"reason", critical->second}};
      }
    }
    if (control_failed_ && raw == "running" && !safety_pending) {
      blackboard_->set("mission_state", std::string("failed"));
      control_failed_ = false;
      if (!failure_.is_null()) {
        if (!failure_.contains("reason") || failure_["reason"].get<std::string>().empty()) {
          const auto terminal_error = failure_.value("terminal_error", std::string{});
          if (!terminal_error.empty()) {failure_["reason"] = terminal_error;}
        }
        failure_["message"] = missionFailureMessage(failure_);
      }
      event_ = {{"type", "mission_finished"}, {"result", "failed"}};
      if (!failure_.is_null()) {event_["message"] = failure_["message"];}
      events_.push_back(event_);
      run_open_ = false;
      publish("failed");
      return;
    }
    control_failed_ = false;
    if (state == "running" && previous_state_ != "running" &&
      previous_state_ != "paused") {
      if (events_.empty()) {
        event_ = {{"type", "mission_started"}};
        events_.push_back(event_);
      }
      publish(state);
      return;
    }
    if (state != previous_state_) {
      if (state == "completed" || state == "aborted") {
        event_ = {{"type", "mission_finished"}, {"result", state}};
        events_.push_back(event_);
        run_open_ = false;
      } else if (state == "paused" || (state == "running" && previous_state_ == "paused")) {
        event_ = {{"type", state == "paused" ? "mission_paused" : "mission_resumed"}};
        events_.push_back(event_);
      } else {
        event_ = nullptr;
      }
      publish(state.empty() ? "idle" : state);
    } else if (dirty_) {
      publish(state.empty() ? "idle" : state);
    }
  }

private:
  void publish(const std::string & state)
  {
    nlohmann::json snapshot = {
      {"session_id", session_id_}, {"run_id", run_number_},
      {"sequence", ++sequence_}, {"timestamp", node_->now().seconds()},
      {"state", state},
      {"event", event_}, {"events", events_}, {"active_actions", active_},
      {"completed_actions", completed_order_}, {"failed_actions", failed_},
      {"failure", failure_}, {"safety", safety_}
    };
    if (!failure_.is_null() && failure_.contains("message")) {
      snapshot["message"] = failure_["message"];
    }
    std_msgs::msg::String message;
    message.data = snapshot.dump();
    publisher_->publish(message);
    previous_state_ = state;
    dirty_ = false;
    event_ = nullptr;
    events_.clear();
  }

  BT::Blackboard::Ptr blackboard_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<sura_bt::MissionFailureReasons> failure_reasons_;
  std::shared_ptr<sura_safety::CriticalSafetyState> critical_safety_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string session_id_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
  std::set<std::string> action_names_, active_, completed_, failed_;
  std::vector<std::string> completed_order_;
  std::vector<nlohmann::json> events_;
  nlohmann::json event_ = nullptr;
  nlohmann::json failure_ = nullptr;
  nlohmann::json safety_ = nullptr;
  std::string previous_state_ = "idle";
  uint64_t run_number_ = 0, sequence_ = 0;
  bool dirty_ = false, control_failed_ = false, run_open_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  previous_logging_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(missionLoggingOutputHandler);

  auto ros_node = std::make_shared<rclcpp::Node>("sura_bt_runner");

  // Keep the positional syntax working for existing scripts, while exposing
  // robot_namespace as a ROS parameter for launch files and --ros-args -p.
  const auto legacy_robot_namespaces =
    getArgumentValue(argc, argv, "robot_namespace", "");
  ros_node->declare_parameter<std::string>("robot_namespace", legacy_robot_namespaces);
  ros_node->declare_parameter<std::string>(
    "mission_robot_profiles_file",
    "tools/mission_generator/generated/mission_robot_profiles.json");
  ros_node->declare_parameter<double>("available_axis_relative_threshold", 0.10);
  const auto configured_robots = ros_node->get_parameter("robot_namespace").as_string();
  const auto robots = configured_robots.empty() ?
    discoverRobotsFromDiagnostics(ros_node) :
    sura_bt::parseRobotNamespaces(configured_robots);
  const bool multi_robot = robots.size() > 1;
  const std::string & robot_namespace = robots.front();

  RCLCPP_INFO(
    ros_node->get_logger(),
    "robot_namespace='%s', robots='%s'",
    robot_namespace.c_str(), joinRobotNamespaces(robots).c_str());

  const auto profile_path = ros_node->get_parameter("mission_robot_profiles_file").as_string();
  const auto relative_axis_threshold = ros_node->get_parameter(
    "available_axis_relative_threshold").as_double();
  bool profile_dirty = false;
  std::map<std::string, std_msgs::msg::Float64MultiArray> allocation_matrices;
  std::map<std::string, std_msgs::msg::String> rendered_descriptions;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr>
    matrix_subscriptions;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>
    description_subscriptions;
  for (const auto & robot : robots) {
    const auto topic = "/" + robot + "/controller/body_force/allocation_matrix";
    matrix_subscriptions.push_back(ros_node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [robot, &allocation_matrices, &profile_dirty](
        const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        allocation_matrices[robot] = *msg;
        profile_dirty = true;
      }));
    const auto description_topic = "/" + robot + "/robot_description";
    description_subscriptions.push_back(ros_node->create_subscription<std_msgs::msg::String>(
      description_topic, rclcpp::QoS(1).reliable().transient_local(),
      [robot, &rendered_descriptions, &profile_dirty](
        const std_msgs::msg::String::SharedPtr msg) {
        rendered_descriptions[robot] = *msg;
        profile_dirty = true;
      }));
  }
  const auto matrix_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (rclcpp::ok() && (allocation_matrices.size() < robots.size() ||
    rendered_descriptions.size() < robots.size()) &&
    std::chrono::steady_clock::now() < matrix_deadline)
  {
    rclcpp::spin_some(ros_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  auto current_profiles = writeMissionRobotProfiles(
    robots, allocation_matrices, rendered_descriptions, profile_path, relative_axis_threshold);
  profile_dirty = false;
  RCLCPP_INFO(
    ros_node->get_logger(),
    "Wrote mission robot profiles to '%s' (%zu/%zu descriptions, %zu/%zu allocation matrices)",
    profile_path.c_str(), rendered_descriptions.size(), robots.size(),
    allocation_matrices.size(), robots.size());

  const std::string tree_file =
    getTreeFile("sura_bt", "trees/mission.xml");
  std::vector<std::string> safety_tree_files;
  safety_tree_files.reserve(robots.size());
  for (const auto & robot : robots)
  {
    safety_tree_files.push_back(getSafetyTreeFile(robot));
  }
  const std::string fleet_safety_file = multi_robot ?
    getTreeFile("sura_bt", "trees/fleet_safety.xml") : std::string{};
  const std::string autonomous_tree_file =
    getTreeFile("sura_bt", "trees/autonomous_branch.xml");
  const std::string teleop_tree_file =
    getTreeFile("sura_bt", "trees/teleop_branch.xml");

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loading tree: %s",
    tree_file.c_str()
  );
  for (const auto & safety_tree_file : safety_tree_files)
  {
    RCLCPP_INFO(
      ros_node->get_logger(), "Loading safety tree: %s", safety_tree_file.c_str());
  }
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
  auto safety_ask_state = std::make_shared<sura_safety::SafetyAskState>();
  auto mission_completed_actions = std::make_shared<std::unordered_set<std::string>>();
  auto mission_failure_reasons = std::make_shared<sura_bt::MissionFailureReasons>();
  auto critical_safety_state = std::make_shared<sura_safety::CriticalSafetyState>();
  blackboard->set("safety_ask_state", safety_ask_state);
  blackboard->set("mission_completed_actions", mission_completed_actions);
  blackboard->set("mission_failure_reasons", mission_failure_reasons);
  blackboard->set("critical_safety_state", critical_safety_state);
  for (const auto & robot : robots) {
    const auto family = loadRobotProfile(robot).value("family", nlohmann::ordered_json(nullptr));
    if (!family.is_string() || family.get<std::string>().empty()) {
      throw std::runtime_error("Could not determine robot family for '" + robot + "'");
    }
    blackboard->set("robot_family_" + robot, family.get<std::string>());
  }
  const auto set_available_axes = [&blackboard](const nlohmann::ordered_json & profiles) {
    for (const auto & profile : profiles) {
      if (!profile.contains("name") || !profile["name"].is_string()) {continue;}
      std::vector<std::string> axes;
      if (profile.contains("available_axes") && profile["available_axes"].is_array()) {
        for (const auto & axis : profile["available_axes"]) {
          if (axis.is_string()) {axes.push_back(axis.get<std::string>());}
        }
      }
      blackboard->set("robot_available_axes_" + profile["name"].get<std::string>(), axes);
    }
  };
  set_available_axes(current_profiles);

  const auto switchable_controllers =
    loadSwitchableControllers(robot_namespace);

  blackboard->set("switchable_controllers", switchable_controllers);

  RCLCPP_INFO(
    ros_node->get_logger(),
    "Loaded %zu switchable controllers from ros2_control params",
    switchable_controllers.size());

  sura_safety::configureSafetyBlackboard(
    ros_node,
    blackboard,
    robot_namespace);

  if (multi_robot)
  {
    blackboard->set("robot_namespaces", joinRobotNamespaces(robots));
    blackboard->set("robot_namespace_" + robot_namespace, robot_namespace);
    blackboard->set("mission_control_" + robot_namespace, std::string("run"));
    blackboard->set(
      "diagnostics_monitor_" + robot_namespace,
      blackboard->get<std::shared_ptr<sura_safety::DiagnosticsMonitor>>("diagnostics_monitor"));
    blackboard->set("switchable_controllers_" + robot_namespace, switchable_controllers);

    for (std::size_t index = 1; index < robots.size(); ++index)
    {
      const auto & robot = robots[index];
      blackboard->set("robot_namespace_" + robot, robot);
      blackboard->set("mission_control_" + robot, std::string("run"));
      auto robot_blackboard = BT::Blackboard::create();
      sura_safety::configureSafetyBlackboard(ros_node, robot_blackboard, robot);
      blackboard->set(
        "diagnostics_monitor_" + robot,
        robot_blackboard->get<std::shared_ptr<sura_safety::DiagnosticsMonitor>>(
          "diagnostics_monitor"));
      blackboard->set("switchable_controllers_" + robot, loadSwitchableControllers(robot));
    }

    RCLCPP_WARN(
      ros_node->get_logger(),
      "The current mission XML uses robot_namespace='%s' for its existing commands; "
      "safety is active for all selected robots",
      robot_namespace.c_str());
  }

  auto build_tree = [&]() {
      BT::BehaviorTreeFactory factory;

      sura_safety::registerSafetyNodes(factory);
      sura_bt::registerNodes(factory);

      if (multi_robot)
      {
        for (std::size_t index = 0; index < robots.size(); ++index)
        {
          factory.registerBehaviorTreeFromText(
            sura_bt::renamedSafetyTreeXml(safety_tree_files[index], robots[index]));
        }
        factory.registerBehaviorTreeFromText(
          sura_bt::expandedFleetSafetyXml(fleet_safety_file, robots));
      }
      else
      {
        factory.registerBehaviorTreeFromText(
          sura_bt::resolvedSafetyTreeXml(safety_tree_files.front(), robots.front()));
      }
      factory.registerBehaviorTreeFromText(
        sura_bt::checkpointedAutonomousTreeXml(autonomous_tree_file));
      factory.registerBehaviorTreeFromFile(teleop_tree_file);
      factory.registerBehaviorTreeFromFile(tree_file);

      return factory.createTree("Mission", blackboard);
    };

  auto tree = build_tree();
  MissionProgress mission_progress(
    ros_node, blackboard, mission_failure_reasons, critical_safety_state);
  mission_progress.attach(tree, autonomous_tree_file);

  auto publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
  std::vector<BT::Tree> retired_trees;

  sura_bt::SafetyAskCoordinator safety_ask_coordinator(
    ros_node, blackboard, safety_ask_state, mission_completed_actions,
    autonomous_tree_file, profile_path);

  auto reload_service = ros_node->create_service<std_srvs::srv::Trigger>(
    "~/reload_autonomous_tree",
    [&](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::string mission_state;
      if (blackboard->get("mission_state", mission_state) &&
        mission_state == "running")
      {
        response->success = false;
        response->message =
          "Cannot reload AutonomousBranch while mission_state='" + mission_state + "'";
        RCLCPP_WARN(
          ros_node->get_logger(),
          "%s",
          response->message.c_str());
        return;
      }

      try
      {
        safety_ask_coordinator.validateReload();
        auto new_tree = build_tree();
        tree.haltTree();
        publisher_zmq.reset();
        retired_trees.emplace_back(std::move(tree));
        tree = std::move(new_tree);
        mission_progress.attach(tree, autonomous_tree_file);
        publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
        safety_ask_coordinator.onReloadSuccess();

        response->success = true;
        response->message = "AutonomousBranch reloaded";
        RCLCPP_INFO(
          ros_node->get_logger(),
          "%s",
          response->message.c_str());
      }
      catch (const std::exception & exception)
      {
        response->success = false;
        response->message =
          std::string("Failed to reload AutonomousBranch: ") + exception.what();
        RCLCPP_ERROR(
          ros_node->get_logger(),
          "%s",
          response->message.c_str());
      }
    });

  rclcpp::Rate rate(10.0);

  while (rclcpp::ok())
  {
    tree.tickRoot();
    safety_ask_coordinator.publishPending();
    mission_progress.afterTick(safety_ask_coordinator.pending());

    rclcpp::spin_some(ros_node);
    if (profile_dirty) {
      try {
        current_profiles = writeMissionRobotProfiles(
          robots, allocation_matrices, rendered_descriptions, profile_path, relative_axis_threshold);
        set_available_axes(current_profiles);
        profile_dirty = false;
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          ros_node->get_logger(), "Could not refresh mission robot profiles: %s",
          exception.what());
      }
    }

    rate.sleep();
  }

  publisher_zmq.reset();
  tree.haltTree();
  for (auto & retired_tree : retired_trees)
  {
    retired_tree.haltTree();
  }

  rclcpp::shutdown();
  return 0;
}
