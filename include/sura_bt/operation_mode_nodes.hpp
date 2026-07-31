#pragma once

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/control_node.h"

namespace sura_bt
{

class TeleopRequested : public BT::ConditionNode
{
public:
  TeleopRequested(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class AutonomousRequested : public BT::ConditionNode
{
public:
  AutonomousRequested(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool was_requested_{false};
};

class VariableSet : public BT::SyncActionNode
{
public:
  VariableSet(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class VariableIs : public BT::ConditionNode
{
public:
  VariableIs(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class VariableIsNot : public BT::ConditionNode
{
public:
  VariableIsNot(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MissionControl : public BT::ControlNode
{
public:
  MissionControl(
    const std::string & name,
    const BT::NodeConfiguration & config);

  void halt() override;

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  size_t current_child_idx_{0};
};

class MissionCompleted : public BT::StatefulActionNode
{
public:
  MissionCompleted(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool isCompleted() const;
};

class TeleopSession : public BT::StatefulActionNode
{
public:
  TeleopSession(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

}  // namespace sura_bt
