#include "sura_bt/fleet_tree.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "tinyxml2.h"

namespace sura_bt
{
namespace
{

std::string trim(std::string value)
{
  const auto not_space = [](unsigned char character) {return !std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string normalizedNamespace(std::string value)
{
  value = trim(value);
  while (!value.empty() && value.front() == '/') {value.erase(value.begin());}
  while (!value.empty() && value.back() == '/') {value.pop_back();}
  if (value.empty() ||
    !std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isalnum(character) || character == '_' || character == '-';
    }))
  {
    throw std::invalid_argument("Invalid robot_namespace: '" + value + "'");
  }
  return value;
}

void loadXml(const std::string & file, tinyxml2::XMLDocument & document)
{
  if (document.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error("Could not load XML '" + file + "': " + document.ErrorStr());
  }
}

void resolveRobotNamespaceReferences(
  tinyxml2::XMLNode * node,
  const std::string & robot_namespace)
{
  if (!node) {return;}

  if (auto * element = node->ToElement())
  {
    for (auto * attribute = element->FirstAttribute(); attribute;
      attribute = attribute->Next())
    {
      if (std::string(attribute->Value()) == "{robot_namespace}")
      {
        element->SetAttribute(attribute->Name(), robot_namespace.c_str());
      }
    }
  }

  for (auto * child = node->FirstChild(); child; child = child->NextSibling())
  {
    resolveRobotNamespaceReferences(child, robot_namespace);
  }
}

std::string xmlText(const tinyxml2::XMLDocument & document)
{
  tinyxml2::XMLPrinter printer;
  document.Print(&printer);
  return printer.CStr();
}

}  // namespace

std::vector<std::string> parseRobotNamespaces(const std::string & value)
{
  std::vector<std::string> robots;
  std::set<std::string> seen;
  std::size_t begin = 0;
  while (begin < value.size())
  {
    const auto end = value.find(',', begin);
    const auto item = normalizedNamespace(value.substr(begin, end - begin));
    if (!seen.insert(item).second)
    {
      throw std::invalid_argument("Duplicate robot_namespace: '" + item + "'");
    }
    robots.push_back(item);
    if (end == std::string::npos) {break;}
    begin = end + 1;
    if (begin == value.size())
    {
      throw std::invalid_argument("Trailing comma in robot_namespace list");
    }
  }
  return robots;
}

std::vector<std::string> robotsFromDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & diagnostics)
{
  std::set<std::string> robots;
  for (const auto & status : diagnostics.status)
  {
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {continue;}
    for (const auto & value : status.values)
    {
      if (value.key != "source_name") {continue;}
      const auto first = value.value.find_first_not_of('/');
      if (first == std::string::npos) {continue;}
      const auto slash = value.value.find('/', first);
      if (slash == std::string::npos) {continue;}
      try
      {
        robots.insert(normalizedNamespace(value.value.substr(first, slash - first)));
      }
      catch (const std::invalid_argument &)
      {
        // Ignore sources that do not have a valid robot namespace.
      }
    }
  }
  return {robots.begin(), robots.end()};
}

std::string safetyTreeId(const std::string & robot_namespace)
{
  return "SafetyBranch_" + normalizedNamespace(robot_namespace);
}

std::string resolvedSafetyTreeXml(
  const std::string & file,
  const std::string & robot_namespace)
{
  tinyxml2::XMLDocument document;
  loadXml(file, document);
  const auto name = normalizedNamespace(robot_namespace);
  resolveRobotNamespaceReferences(document.RootElement(), name);
  auto * root = document.FirstChildElement("root");
  auto * tree = root ? root->FirstChildElement("BehaviorTree") : nullptr;
  if (!tree || tree->NextSiblingElement("BehaviorTree") ||
    !tree->Attribute("ID", "SafetyBranch"))
  {
    throw std::runtime_error("Expected a single SafetyBranch in '" + file + "'");
  }

  const auto robot_tree_id = safetyTreeId(name);
  tree->SetAttribute("ID", robot_tree_id.c_str());

  auto * robot_tree = document.NewElement("BehaviorTree");
  robot_tree->SetAttribute("ID", "SafetyBranch");
  auto * robot_subtree = document.NewElement("SubTree");
  robot_subtree->SetAttribute("ID", robot_tree_id.c_str());
  robot_subtree->SetAttribute("name", name.c_str());
  robot_subtree->SetAttribute("__shared_blackboard", "true");
  robot_tree->InsertEndChild(robot_subtree);
  root->InsertEndChild(robot_tree);
  return xmlText(document);
}

std::string renamedSafetyTreeXml(
  const std::string & file,
  const std::string & robot_namespace)
{
  tinyxml2::XMLDocument document;
  loadXml(file, document);
  auto * root = document.FirstChildElement("root");
  auto * tree = root ? root->FirstChildElement("BehaviorTree") : nullptr;
  if (!tree || tree->NextSiblingElement("BehaviorTree") ||
    !tree->Attribute("ID", "SafetyBranch"))
  {
    throw std::runtime_error("Expected a single SafetyBranch in '" + file + "'");
  }
  resolveRobotNamespaceReferences(document.RootElement(), normalizedNamespace(robot_namespace));
  tree->SetAttribute("ID", safetyTreeId(robot_namespace).c_str());
  return xmlText(document);
}

std::string checkpointedAutonomousTreeXml(const std::string & file)
{
  tinyxml2::XMLDocument document;
  loadXml(file, document);
  auto * root = document.FirstChildElement("root");
  auto * tree = root ? root->FirstChildElement("BehaviorTree") : nullptr;
  if (!tree || tree->NextSiblingElement("BehaviorTree") ||
    !tree->Attribute("ID", "AutonomousBranch"))
  {
    throw std::runtime_error("Expected one AutonomousBranch in '" + file + "'");
  }

  const std::set<std::string> checkpointed_actions = {
    "Delay", "GoToPoseAction", "SendVelocity", "SendWrench",
    "SurfaceAction", "TakePhoto"};
  std::map<std::string, unsigned> occurrences;
  std::function<void(tinyxml2::XMLNode *)> visit = [&](tinyxml2::XMLNode * parent) {
    for (auto * node = parent->FirstChild(); node; )
    {
      auto * next = node->NextSibling();
      auto * element = node->ToElement();
      if (element && checkpointed_actions.count(element->Name()) != 0)
      {
        std::ostringstream signature;
        signature << element->Name();
        for (auto * attribute = element->FirstAttribute(); attribute; attribute = attribute->Next())
        {
          signature << '|' << attribute->Name() << '=' << attribute->Value();
        }
        const auto base = signature.str();
        const auto occurrence = occurrences[base]++;
        const auto key = std::to_string(std::hash<std::string>{}(base)) + "_" +
          std::to_string(occurrence);
        auto * checkpoint = document.NewElement("MissionCheckpoint");
        checkpoint->SetAttribute("key", key.c_str());
        checkpoint->SetAttribute("name", (std::string("checkpoint_") +
          (element->Attribute("name") ? element->Attribute("name") : element->Name())).c_str());
        checkpoint->InsertEndChild(element->DeepClone(&document));
        parent->InsertAfterChild(node, checkpoint);
        parent->DeleteChild(node);
      }
      else if (element)
      {
        visit(element);
      }
      node = next;
    }
  };
  visit(tree);
  return xmlText(document);
}

std::string expandedFleetSafetyXml(
  const std::string & file,
  const std::vector<std::string> & robot_namespaces)
{
  if (robot_namespaces.size() < 2)
  {
    throw std::invalid_argument("Fleet safety requires at least two robots");
  }
  tinyxml2::XMLDocument document;
  loadXml(file, document);
  auto * root = document.FirstChildElement("root");
  auto * tree = root ? root->FirstChildElement("BehaviorTree") : nullptr;
  auto * parallel = tree ? tree->FirstChildElement("ReactiveParallel") : nullptr;
  auto * placeholder = parallel ? parallel->FirstChildElement("RobotSafetyBranches") : nullptr;
  if (!tree || !tree->Attribute("ID", "SafetyBranch") || !placeholder)
  {
    throw std::runtime_error("Invalid fleet safety template '" + file + "'");
  }

  tinyxml2::XMLNode * previous = placeholder;
  for (const auto & robot : robot_namespaces)
  {
    const auto name = normalizedNamespace(robot);
    auto * subtree = document.NewElement("SubTree");
    subtree->SetAttribute("ID", safetyTreeId(name).c_str());
    subtree->SetAttribute("name", name.c_str());
    subtree->SetAttribute("robot_namespace", ("robot_namespace_" + name).c_str());
    subtree->SetAttribute("ros_node", "ros_node");
    subtree->SetAttribute("diagnostics_monitor", ("diagnostics_monitor_" + name).c_str());
    subtree->SetAttribute("switchable_controllers", ("switchable_controllers_" + name).c_str());
    subtree->SetAttribute("mission_control", ("mission_control_" + name).c_str());
    subtree->SetAttribute("mission_state", "mission_state");
    subtree->SetAttribute("safety_ask_state", "safety_ask_state");
    parallel->InsertAfterChild(previous, subtree);
    previous = subtree;
  }
  parallel->DeleteChild(placeholder);
  return xmlText(document);
}

}  // namespace sura_bt
