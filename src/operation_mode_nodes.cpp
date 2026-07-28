#include "sura_bt/operation_mode_nodes.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

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

std::string namespacedPath(const BT::NodeConfiguration & config, const std::string & suffix)
{
  if (!suffix.empty() && suffix.front() == '/')
  {
    return suffix;
  }

  const auto robot_namespace =
    stripSlashes(config.blackboard->get<std::string>("robot_namespace"));
  const auto normalized_suffix = stripSlashes(suffix);

  if (robot_namespace.empty())
  {
    return "/" + normalized_suffix;
  }

  return "/" + robot_namespace + "/" + normalized_suffix;
}

struct BoolTopicState
{
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
  std::mutex mutex;
  std_msgs::msg::Bool::SharedPtr last_msg;
};

std::mutex bool_topics_mutex;
std::unordered_map<std::string, std::shared_ptr<BoolTopicState>> bool_topics;

std::shared_ptr<BoolTopicState> getBoolTopicState(
  const BT::NodeConfiguration & config,
  const std::string & topic)
{
  std::lock_guard<std::mutex> topics_lock(bool_topics_mutex);
  auto it = bool_topics.find(topic);
  if (it != bool_topics.end())
  {
    return it->second;
  }

  auto state = std::make_shared<BoolTopicState>();
  const auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  state->sub = ros_node->create_subscription<std_msgs::msg::Bool>(
    topic,
    rclcpp::SystemDefaultsQoS(),
    [state](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->last_msg = msg;
    });

  bool_topics.emplace(topic, state);

  RCLCPP_INFO(
    ros_node->get_logger(),
    "[sura_bt] Operation mode subscribed to %s",
    topic.c_str());

  return state;
}

bool readAutonomousRequested(
  const BT::NodeConfiguration & config,
  const std::string & topic)
{
  const auto state = getBoolTopicState(config, namespacedPath(config, topic));
  std_msgs::msg::Bool::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    msg = state->last_msg;
  }

  return msg && msg->data;
}

}  // namespace

TeleopRequested::TeleopRequested(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList TeleopRequested::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic", "teleop/autonomous_enabled", "Autonomous mode topic")
  };
}

BT::NodeStatus TeleopRequested::tick()
{
  const auto topic = getInput<std::string>("topic").value_or("teleop/autonomous_enabled");

  if (!readAutonomousRequested(config(), topic))
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

AutonomousRequested::AutonomousRequested(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList AutonomousRequested::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic", "teleop/autonomous_enabled", "Autonomous mode topic")
  };
}

BT::NodeStatus AutonomousRequested::tick()
{
  const auto topic = getInput<std::string>("topic").value_or("teleop/autonomous_enabled");

  const bool requested = readAutonomousRequested(config(), topic);
  if (!requested)
  {
    was_requested_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (!was_requested_)
  {
    std::string mission_state;
    if (config().blackboard->get("mission_state", mission_state) &&
      mission_state == "completed")
    {
      config().blackboard->set("mission_state", std::string{"requested"});
    }
  }

  was_requested_ = true;
  return BT::NodeStatus::SUCCESS;
}

VariableSet::VariableSet(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList VariableSet::providedPorts()
{
  return {
    BT::InputPort<std::string>("key"),
    BT::InputPort<std::string>("value")
  };
}

BT::NodeStatus VariableSet::tick()
{
  const auto key = getInput<std::string>("key");
  const auto value = getInput<std::string>("value");
  if (!key || key.value().empty() || !value)
  {
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set(key.value(), value.value());
  return BT::NodeStatus::SUCCESS;
}

VariableIs::VariableIs(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList VariableIs::providedPorts()
{
  return {
    BT::InputPort<std::string>("key"),
    BT::InputPort<std::string>("value")
  };
}

BT::NodeStatus VariableIs::tick()
{
  const auto key = getInput<std::string>("key");
  const auto expected_value = getInput<std::string>("value");
  if (!key || key.value().empty() || !expected_value)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::string current_value;
  const bool matches =
    config().blackboard->get(key.value(), current_value) &&
    current_value == expected_value.value();

  return matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

VariableIsNot::VariableIsNot(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList VariableIsNot::providedPorts()
{
  return {
    BT::InputPort<std::string>("key"),
    BT::InputPort<std::string>("value")
  };
}

BT::NodeStatus VariableIsNot::tick()
{
  const auto key = getInput<std::string>("key");
  const auto expected_value = getInput<std::string>("value");
  if (!key || key.value().empty() || !expected_value)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::string current_value;
  const bool matches =
    config().blackboard->get(key.value(), current_value) &&
    current_value == expected_value.value();

  return matches ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

MissionCompleted::MissionCompleted(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList MissionCompleted::providedPorts()
{
  return {};
}

BT::NodeStatus MissionCompleted::onStart()
{
  return isCompleted() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::NodeStatus MissionCompleted::onRunning()
{
  return isCompleted() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

void MissionCompleted::onHalted()
{
}

bool MissionCompleted::isCompleted() const
{
  std::string mission_state;
  return config().blackboard->get("mission_state", mission_state) &&
    mission_state == "completed";
}

TeleopSession::TeleopSession(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList TeleopSession::providedPorts()
{
  return {};
}

BT::NodeStatus TeleopSession::onStart()
{
  config().blackboard->set("teleop_session_active", true);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TeleopSession::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void TeleopSession::onHalted()
{
  config().blackboard->set("teleop_session_active", false);
}

}  // namespace sura_bt
