#include "sura_bt/register_nodes.hpp"

#include <string>

#include "sura_bt/camera_nodes.hpp"
#include "sura_bt/command_nodes.hpp"
#include "sura_bt/controller_switch_nodes.hpp"
#include "sura_bt/operation_mode_nodes.hpp"
#include "sura_bt/sura_actions_nodes.hpp"

namespace sura_bt
{
namespace
{

template<typename NodeT>
void registerNodeWithDescription(
  BT::BehaviorTreeFactory & factory,
  const std::string & node_id)
{
  factory.registerNodeType<NodeT>(node_id);
  factory.addDescriptionToManifest(node_id, NodeT::main_description());
}

}  // namespace

void registerNodes(BT::BehaviorTreeFactory & factory)
{
  registerNodeWithDescription<SendWrench>(
    factory, "SendWrench");

  registerNodeWithDescription<SendVelocity>(
    factory, "SendVelocity");

  registerNodeWithDescription<ComputeAreaRecoveryForce>(
    factory, "ComputeAreaRecoveryForce");

  registerNodeWithDescription<TakePhoto>(
    factory, "TakePhoto");

  registerNodeWithDescription<ActivateControllers>(
    factory, "ActivateControllers");

  registerNodeWithDescription<DeactivateControllers>(
    factory, "DeactivateControllers");

  registerNodeWithDescription<DeactivateSystem>(
    factory, "DeactivateSystem");

  registerNodeWithDescription<SetControllerInterlock>(
    factory, "SetControllerInterlock");

  registerNodeWithDescription<SurfaceAction>(
    factory, "SurfaceAction");

  registerNodeWithDescription<GoToPoseAction>(
    factory, "GoToPoseAction");


  registerNodeWithDescription<TeleopRequested>(
    factory, "TeleopRequested");

  registerNodeWithDescription<AutonomousRequested>(
    factory, "AutonomousRequested");

  registerNodeWithDescription<VariableSet>(
    factory, "VariableSet");

  registerNodeWithDescription<VariableIs>(
    factory, "VariableIs");

  registerNodeWithDescription<VariableIsNot>(
    factory, "VariableIsNot");

  registerNodeWithDescription<ReactiveParallel>(
    factory, "ReactiveParallel");

  registerNodeWithDescription<MissionCheckpoint>(
    factory, "MissionCheckpoint");

  registerNodeWithDescription<MissionControl>(
    factory, "MissionControl");

  registerNodeWithDescription<MissionCompleted>(
    factory, "MissionCompleted");

  registerNodeWithDescription<TeleopSession>(
    factory, "TeleopSession");
}

}  // namespace sura_bt
