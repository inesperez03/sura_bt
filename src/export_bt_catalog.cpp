#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/demangle_util.h"
#include "nlohmann/json.hpp"
#include "sura_bt/register_nodes.hpp"

namespace
{

std::string nodeTypeName(const BT::NodeType type)
{
  switch (type)
  {
    case BT::NodeType::ACTION:
      return "Action";
    case BT::NodeType::CONDITION:
      return "Condition";
    case BT::NodeType::CONTROL:
      return "Control";
    case BT::NodeType::DECORATOR:
      return "Decorator";
    case BT::NodeType::SUBTREE:
      return "SubTree";
    case BT::NodeType::UNDEFINED:
      return "Undefined";
  }

  return "Unknown";
}

std::string portDirectionName(const BT::PortDirection direction)
{
  switch (direction)
  {
    case BT::PortDirection::INPUT:
      return "input";
    case BT::PortDirection::OUTPUT:
      return "output";
    case BT::PortDirection::INOUT:
      return "inout";
  }

  return "unknown";
}

nlohmann::json portToJson(
  const BT::PortInfo & port)
{
  nlohmann::json port_json;
  port_json["direction"] = portDirectionName(port.direction());
  port_json["type"] = BT::demangle(port.type());
  port_json["description"] = port.description();
  return port_json;
}

nlohmann::json manifestToJson(const BT::TreeNodeManifest & manifest)
{
  nlohmann::json node_json;
  node_json["type"] = nodeTypeName(manifest.type);
  node_json["description"] = manifest.description;
  node_json["ports"] = nlohmann::json::object();

  std::vector<std::string> port_names;
  port_names.reserve(manifest.ports.size());
  for (const auto & port_it : manifest.ports)
  {
    port_names.push_back(port_it.first);
  }

  std::sort(port_names.begin(), port_names.end());

  for (const auto & port_name : port_names)
  {
    node_json["ports"][port_name] = portToJson(manifest.ports.at(port_name));
  }

  return node_json;
}

nlohmann::json buildCatalogJson(const BT::BehaviorTreeFactory & factory)
{
  nlohmann::json catalog;
  catalog["nodes"] = nlohmann::json::object();

  std::vector<std::string> node_ids;
  node_ids.reserve(factory.manifests().size());
  for (const auto & manifest_it : factory.manifests())
  {
    node_ids.push_back(manifest_it.first);
  }

  std::sort(node_ids.begin(), node_ids.end());

  for (const auto & node_id : node_ids)
  {
    const bool builtin = factory.builtinNodes().count(node_id) > 0;
    if (builtin)
    {
      continue;
    }

    catalog["nodes"][node_id] =
      manifestToJson(factory.manifests().at(node_id));
  }

  return catalog;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string output_path =
    argc > 1 ? argv[1] : "tools/mission_generator/generated/bt_catalog.json";

  BT::BehaviorTreeFactory factory;
  sura_bt::registerNodes(factory);

  const auto catalog = buildCatalogJson(factory);

  const auto parent_path = std::filesystem::path(output_path).parent_path();
  if (!parent_path.empty())
  {
    std::filesystem::create_directories(parent_path);
  }

  std::ofstream output(output_path);
  if (!output)
  {
    std::cerr << "Failed to open output file: " << output_path << std::endl;
    return 1;
  }

  output << catalog.dump(2) << std::endl;

  std::cout << "BT catalog exported to: " << output_path << std::endl;
  std::cout << "Nodes exported: " << catalog["nodes"].size() << std::endl;

  return 0;
}
