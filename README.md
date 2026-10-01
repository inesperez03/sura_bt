# Mission robot profiles

When `bt_runner` starts, it writes `tools/mission_generator/generated/mission_robot_profiles.json` for the
robots selected by `robot_namespace` or discovered from diagnostics. The JSON contains
robot name, sensor names, non-thruster actuator names, non-broadcaster controllers,
family and `available_axes`. Axes are derived from the body-force allocation matrix
and each thruster's calibrated force range. Axes below 10% of the strongest axis in
the same force/torque group, or below 1 N / 0.1 Nm, are omitted. Set the ROS parameter
`available_axis_relative_threshold` to change the relative cutoff.

## Safety decision workflow

`SafetyErrorAsk` pauses the autonomous BT subtree and publishes a transient-local
JSON event on `/sura_bt_runner/safety_ask`. The event contains the robot/diagnostic
source, diagnostic detail, mission XML path, configured robot profile path and
completed action checkpoint keys and a checkpointed tree snapshot.
`SafetyAskCoordinator` owns the ROS event, pause handshake and reload guards;
`bt_runner` only ticks the tree and asks the coordinator to validate a reload.
The safety subtree remains active while the autonomous mission waits.

Groot's existing Mission Assistant chat starts
`tools/mission_generator/safety_dialogue.py` when it receives the event. The
agent reads the active XML and robot profiles, extracts completed actions from
the checkpointed snapshot, and combines them with the diagnostic and conversation
history. It asks the operator for a solution tailored to the
remaining mission; the operator replies in the same chat. It can show a photo
from a discovered camera topic of that robot, request missing parameters, or confirm that the
mission should end. There are no separate safety action buttons. The chat's
**Borrar historial** button clears visible messages and both conversation memories;
a pending safety incident remains paused with its mission context intact.

After the operator confirms a supported continuation, the agent generates a
complete revised `AutonomousBranch`. Required inputs are checked before the
file is written. `bt_runner` refuses a safety reload if the XML is unchanged or
if any completed checkpoint has moved or changed. If the reload fails, the
agent restores the previous XML and leaves the mission paused. After a
successful reload, the agent calls `/sura_bt_runner/resume_safety_ask`; an
explicitly confirmed termination calls `/sura_bt_runner/abort_safety_ask`.
An interrupted action starts again after resume; completed checkpointed actions
are skipped. The checkpoint set is in memory and is lost when bt_runner restarts.

`src/sura_bt/scripts/safety_ask_bridge.py` handles only local ROS event, camera
and service operations. The mission agent uses the existing Gemini configuration
in `tools/mission_generator/.env`; the XML, profiles, diagnostic and dialogue
are sent to the selected Gemini model. Camera images are displayed locally in
Groot and are not sent to the model. If Gemini, ROS or the camera is unavailable,
the mission remains paused and the chat shows the error.
