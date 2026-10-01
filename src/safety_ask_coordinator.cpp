#include "sura_bt/safety_ask_coordinator.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "tinyxml2.h"

#include "sura_bt/fleet_tree.hpp"

namespace sura_bt
{

SafetyAskCoordinator::SafetyAskCoordinator(
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard,
  std::shared_ptr<sura_safety::SafetyAskState> state,
  std::shared_ptr<std::unordered_set<std::string>> completed_actions,
  std::string autonomous_tree_file,
  std::string robot_profiles_file)
: node_(node), blackboard_(blackboard), state_(std::move(state)),
  completed_actions_(std::move(completed_actions)),
  autonomous_tree_file_(std::move(autonomous_tree_file)),
  robot_profiles_file_(std::move(robot_profiles_file))
{
  event_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "~/safety_ask", rclcpp::QoS(1).reliable().transient_local());
  resume_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/resume_safety_ask",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      resume(response);
    });
  abort_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/abort_safety_ask",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      abort(response);
    });
}

bool SafetyAskCoordinator::pending() const
{
  return state_->pending();
}

std::string SafetyAskCoordinator::readAutonomousXml() const
{
  std::ifstream stream(autonomous_tree_file_);
  if (!stream) {
    throw std::runtime_error("Could not read " + autonomous_tree_file_);
  }
  return std::string(std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>());
}

SafetyAskCoordinator::CheckpointMap SafetyAskCoordinator::checkpointActions(
  const std::string & checkpointed_xml)
{
  CheckpointMap actions;
  tinyxml2::XMLDocument document;
  if (document.Parse(checkpointed_xml.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Could not parse checkpointed AutonomousBranch");
  }
  std::function<void(const tinyxml2::XMLElement *, const std::string &)> visit =
    [&](const tinyxml2::XMLElement * element, const std::string & path) {
      unsigned index = 0;
      for (auto * child = element->FirstChildElement(); child;
        child = child->NextSiblingElement(), ++index) {
        std::ostringstream segment;
        segment << std::setw(8) << std::setfill('0') << index;
        const auto child_path = path + "/" + segment.str() + ":" + child->Name();
        if (std::string(child->Name()) == "MissionCheckpoint") {
          const char * key = child->Attribute("key");
          if (key && child->FirstChildElement()) {
            tinyxml2::XMLPrinter printer;
            child->FirstChildElement()->Accept(&printer);
            actions.emplace(key, std::make_pair(child_path, printer.CStr()));
          }
        }
        visit(child, child_path);
      }
    };
  if (document.RootElement()) {
    visit(document.RootElement(), "");
  }
  return actions;
}

SafetyAskCoordinator::CheckpointMap SafetyAskCoordinator::checkpointActionsFromFile() const
{
  return checkpointActions(checkpointedAutonomousTreeXml(autonomous_tree_file_));
}

void SafetyAskCoordinator::validateCompletedActions(
  const CheckpointMap & original,
  const CheckpointMap & candidate,
  const std::unordered_set<std::string> & completed_keys)
{
  for (const auto & key : completed_keys) {
    const auto before = original.find(key);
    const auto after = candidate.find(key);
    if (before == original.end() || after == candidate.end() ||
      before->second != after->second) {
      throw std::runtime_error("Revised mission moved or changed a completed action: " + key);
    }
  }
}

void SafetyAskCoordinator::validateReload() const
{
  if (!pending()) {
    return;
  }
  if (readAutonomousXml() == paused_autonomous_xml_) {
    throw std::runtime_error("Safety decision requires a changed AutonomousBranch before reloading");
  }
  validateCompletedActions(
    paused_checkpoints_, checkpointActionsFromFile(), *completed_actions_);
}

void SafetyAskCoordinator::onReloadSuccess()
{
  if (pending()) {
    tree_reloaded_ = true;
  } else {
    completed_actions_->clear();
  }
}

void SafetyAskCoordinator::publishPending()
{
  if (!pending() || state_->active_source == published_source_) {
    return;
  }
  paused_autonomous_xml_ = readAutonomousXml();
  const auto checkpointed_xml = checkpointedAutonomousTreeXml(autonomous_tree_file_);
  paused_checkpoints_ = checkpointActions(checkpointed_xml);
  nlohmann::json event;
  event["source"] = state_->active_source;
  event["detail"] = state_->active_detail;
  event["mission_file"] = autonomous_tree_file_;
  event["robot_profiles_file"] = std::filesystem::absolute(robot_profiles_file_).string();
  event["completed_actions"] = *completed_actions_;
  event["checkpointed_tree_xml"] = checkpointed_xml;
  std_msgs::msg::String message;
  message.data = event.dump();
  event_publisher_->publish(message);
  published_source_ = state_->active_source;
}

void SafetyAskCoordinator::clearEvent()
{
  event_publisher_->publish(std_msgs::msg::String{});
  published_source_.clear();
}

void SafetyAskCoordinator::resetSnapshot()
{
  tree_reloaded_ = false;
  paused_autonomous_xml_.clear();
  paused_checkpoints_.clear();
  clearEvent();
}

void SafetyAskCoordinator::resume(
  const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
{
  std::string mission_state;
  blackboard_->get("mission_state", mission_state);
  if (!pending() || !tree_reloaded_ || mission_state == "aborted") {
    response->success = false;
    response->message = "A pending safety decision and a successful mission reload are required";
    return;
  }
  state_->resolve();
  blackboard_->set("mission_control", std::string("run"));
  blackboard_->set("mission_state", std::string("running"));
  resetSnapshot();
  response->success = true;
  response->message = "Safety decision resolved; autonomous mission resumed";
}

void SafetyAskCoordinator::abort(
  const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
{
  if (!pending()) {
    response->success = false;
    response->message = "No safety decision is pending";
    return;
  }
  state_->abort();
  blackboard_->set("mission_control", std::string("abort"));
  blackboard_->set("mission_state", std::string("aborted"));
  resetSnapshot();
  response->success = true;
  response->message = "Autonomous mission aborted after safety decision";
}

}  // namespace sura_bt
