#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include "behaviortree_cpp_v3/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_safety/safety_ask_state.hpp"

namespace sura_bt
{

// Owns the runtime handshake for a paused autonomous mission. The mission
// generator decides what to propose; this class only accepts safe transitions.
class SafetyAskCoordinator
{
public:
  using Checkpoint = std::pair<std::string, std::string>;  // tree path, action XML
  using CheckpointMap = std::map<std::string, Checkpoint>;  // checkpoint key -> action

  SafetyAskCoordinator(
    const rclcpp::Node::SharedPtr & node,
    const BT::Blackboard::Ptr & blackboard,
    std::shared_ptr<sura_safety::SafetyAskState> state,
    std::shared_ptr<std::unordered_set<std::string>> completed_actions,
    std::string autonomous_tree_file,
    std::string robot_profiles_file);

  bool pending() const;
  void publishPending();
  void validateReload() const;
  void onReloadSuccess();

  // Pure checkpoint inspection is exposed for contract tests.
  static CheckpointMap checkpointActions(const std::string & checkpointed_xml);
  static void validateCompletedActions(
    const CheckpointMap & original,
    const CheckpointMap & candidate,
    const std::unordered_set<std::string> & completed_keys);

private:
  std::string readAutonomousXml() const;
  CheckpointMap checkpointActionsFromFile() const;
  void clearEvent();
  void resetSnapshot();
  void resume(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response);
  void abort(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response);

  rclcpp::Node::SharedPtr node_;
  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<sura_safety::SafetyAskState> state_;
  std::shared_ptr<std::unordered_set<std::string>> completed_actions_;
  std::string autonomous_tree_file_;
  std::string robot_profiles_file_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;
  std::string published_source_;
  std::string paused_autonomous_xml_;
  CheckpointMap paused_checkpoints_;
  bool tree_reloaded_ = false;
};

}  // namespace sura_bt
