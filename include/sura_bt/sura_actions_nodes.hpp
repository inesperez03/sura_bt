#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "sura_actions/action/surface.hpp"

namespace sura_bt
{

class SurfaceAction : public BT::SyncActionNode
{
public:
  using Surface = sura_actions::action::Surface;
  using GoalHandleSurface = rclcpp_action::ClientGoalHandle<Surface>;

  SurfaceAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string actionName(const BT::NodeConfiguration & config) const;

  rclcpp_action::Client<Surface>::SharedPtr action_client_;
};

}  // namespace sura_bt
