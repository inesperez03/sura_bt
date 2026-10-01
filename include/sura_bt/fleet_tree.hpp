#pragma once

#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace sura_bt
{

std::vector<std::string> parseRobotNamespaces(const std::string & value);

std::vector<std::string> robotsFromDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & diagnostics);

std::string safetyTreeId(const std::string & robot_namespace);

std::string resolvedSafetyTreeXml(
  const std::string & file,
  const std::string & robot_namespace);

std::string renamedSafetyTreeXml(
  const std::string & file,
  const std::string & robot_namespace);

std::string checkpointedAutonomousTreeXml(const std::string & file);

std::string expandedFleetSafetyXml(
  const std::string & file,
  const std::vector<std::string> & robot_namespaces);

}  // namespace sura_bt
