#include "sura_bt/operation_mode_nodes.hpp"

#include "sura_bt/fleet_tree.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
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
      "topic", "teleop/autonomous_enabled", "Topic that indicates whether autonomous mode is requested.")
  };
}

const char * TeleopRequested::main_description()
{
  return "Checks whether teleoperation mode is currently requested.";
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
      "topic", "teleop/autonomous_enabled", "Topic that indicates whether autonomous mode is requested.")
  };
}

const char * AutonomousRequested::main_description()
{
  return "Checks whether autonomous mission execution is currently requested.";
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
      (mission_state == "completed" || mission_state == "aborted" || mission_state == "paused"))
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
    BT::InputPort<std::string>("key", "Blackboard variable name to write."),
    BT::InputPort<std::string>("value", "Value to store in the blackboard variable.")
  };
}

const char * VariableSet::main_description()
{
  return "Stores a mission variable in the behavior tree blackboard.";
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
    BT::InputPort<std::string>("key", "Blackboard variable name to compare."),
    BT::InputPort<std::string>("value", "Expected value of the blackboard variable.")
  };
}

const char * VariableIs::main_description()
{
  return "Checks whether a mission variable matches an expected value.";
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
    BT::InputPort<std::string>("key", "Blackboard variable name to compare."),
    BT::InputPort<std::string>("value", "Value that the blackboard variable must not match.")
  };
}

const char * VariableIsNot::main_description()
{
  return "Checks whether a mission variable differs from a given value.";
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

ReactiveParallel::ReactiveParallel(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::PortsList ReactiveParallel::providedPorts()
{
  return {};
}

const char * ReactiveParallel::main_description()
{
  return "Ticks every child on every cycle, including after a sibling succeeds.";
}

BT::NodeStatus ReactiveParallel::tick()
{
  bool any_running = false;
  bool any_failure = false;
  for (auto * child : children_nodes_)
  {
    const auto status = child->executeTick();
    any_running |= status == BT::NodeStatus::RUNNING;
    any_failure |= status == BT::NodeStatus::FAILURE;
  }
  std::string robot_namespaces;
  if (config().blackboard->get("robot_namespaces", robot_namespaces))
  {
    std::string combined = "run";
    for (const auto & robot : parseRobotNamespaces(robot_namespaces))
    {
      std::string request;
      if (!config().blackboard->get("mission_control_" + robot, request))
      {
        throw BT::RuntimeError("No mission_control state for robot ", robot);
      }
      if (request == "abort")
      {
        combined = "abort";
      }
      else if (request == "ask" && combined != "abort")
      {
        combined = "ask";
      }
      else if (request == "pause" && combined != "abort" && combined != "ask")
      {
        combined = "pause";
      }
      else if (request != "run" && request != "pause" && request != "ask" &&
        request != "abort")
      {
        throw BT::RuntimeError(
          "Unknown mission_control state for robot ", robot, ": ", request);
      }
    }
    config().blackboard->set("mission_control", combined);
  }

  if (any_failure)
  {
    haltChildren();
    return BT::NodeStatus::FAILURE;
  }
  if (any_running)
  {
    return BT::NodeStatus::RUNNING;
  }
  haltChildren();
  return BT::NodeStatus::SUCCESS;
}

MissionCheckpoint::MissionCheckpoint(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList MissionCheckpoint::providedPorts()
{
  return {BT::InputPort<std::string>("key", "Stable identity of an autonomous action.")};
}

const char * MissionCheckpoint::main_description()
{
  return "Remembers a completed autonomous action across an intervention reload.";
}

BT::NodeStatus MissionCheckpoint::tick()
{
  const auto key = getInput<std::string>("key");
  if (!key || key->empty()) {throw BT::RuntimeError("MissionCheckpoint requires key");}
  const auto completed = config().blackboard->get<std::shared_ptr<std::unordered_set<std::string>>>(
    "mission_completed_actions");
  if (completed->count(*key) != 0) {return BT::NodeStatus::SUCCESS;}
  const auto result = child_node_->executeTick();
  if (result == BT::NodeStatus::SUCCESS) {completed->insert(*key);}
  return result;
}

MissionControl::MissionControl(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::PortsList MissionControl::providedPorts()
{
  return {};
}

const char * MissionControl::main_description()
{
  return "Runs mission children through recoverable safety errors and stops on abort.";
}

void MissionControl::halt()
{
  current_child_idx_ = 0;
  BT::ControlNode::halt();
}

BT::NodeStatus MissionControl::tick()
{
  std::string mission_state;
  if (config().blackboard->get("mission_state", mission_state) &&
    mission_state == "aborted")
  {
    current_child_idx_ = 0;
    haltChildren();
    return BT::NodeStatus::FAILURE;
  }

  std::string mission_control;
  if (config().blackboard->get("mission_control", mission_control))
  {
    if (mission_control == "ask")
    {
      haltChildren();
      return BT::NodeStatus::RUNNING;
    }

    if (mission_control == "abort")
    {
      current_child_idx_ = 0;
      haltChildren();
      return BT::NodeStatus::FAILURE;
    }
  }

  while (current_child_idx_ < children_nodes_.size())
  {
    const auto child_status = children_nodes_[current_child_idx_]->executeTick();

    if (child_status == BT::NodeStatus::SUCCESS)
    {
      ++current_child_idx_;
      continue;
    }

    if (child_status == BT::NodeStatus::RUNNING)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (child_status == BT::NodeStatus::FAILURE)
    {
      haltChild(current_child_idx_);
      current_child_idx_ = 0;
      return BT::NodeStatus::FAILURE;
    }
  }

  current_child_idx_ = 0;
  haltChildren();
  return BT::NodeStatus::SUCCESS;
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

const char * MissionCompleted::main_description()
{
  return "Keeps the tree running while the mission is in the completed state.";
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

const char * TeleopSession::main_description()
{
  return "Marks an active teleoperation session while teleop control is running.";
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
