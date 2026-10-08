# AnalyzePhoto

`TakePhoto` guarda un PNG y entrega su ruta absoluta en `image_path`.
`AnalyzePhoto` envía esa imagen y el `prompt` al servidor local de Qwen VL,
escribe la respuesta en el puerto `answer` y la muestra en Mission Assistant.
Si se omite `prompt`, describe brevemente la imagen en español. La hoja sigue
en `RUNNING` durante la inferencia; `timeout` vale 180 s por defecto. Una
imagen inválida, un servidor inaccesible o una respuesta vacía causan
`FAILURE`. Si el árbol detiene la hoja, cancela la petición sin publicar una
respuesta tardía.

```xml
<Sequence>
  <TakePhoto robot_namespace="blueboat" topic="/blueboat/camera/image"
             image_path="{photo_path}"/>
  <AnalyzePhoto robot_namespace="blueboat" image_path="{photo_path}"
                prompt="¿Qué objetos aparecen en la imagen?"
                answer="{photo_answer}"/>
</Sequence>
```

Al ejecutar `AnalyzePhoto`, si el servidor local no responde, el nodo inicia
`start_qwen_vl_cpu.sh` y espera hasta 180 segundos a que esté listo. Si ya hay
un servidor activo, lo reutiliza. En este equipo están instalados `llama-server`
y los pesos oficiales en `.sura/vision/` (directorio ignorado por Git). En otro
equipo, instala una versión reciente de `llama-server`; si no hay modelo en
`.sura/vision/`, el script obtiene `Qwen/Qwen3-VL-2B-Instruct-GGUF:Q4_K_M` y su
proyector al primer arranque. Esa descarga requiere conexión a Internet. Puedes
fijar `SURA_VLM_HOME` para usar otra ubicación local y
`SURA_VLM_ENDPOINT` para configurar otro puerto de loopback. El servidor usa
CPU y solo escucha en `127.0.0.1`. La respuesta se publica además en
`/sura_bt_runner/vision_answers` para el puente de Mission Assistant. El
modelo no controla el robot: cualquier uso posterior de `answer` debe estar
representado explícitamente en el BT.

Después de analizar una foto, puedes escribir preguntas de seguimiento en el
mismo chat de Mission Assistant, por ejemplo «¿De qué color son los objetos?».
El asistente identifica las preguntas sobre la foto y se las envía a Qwen VL
con la última imagen y el diálogo visual reciente. Escribe
`/foto ¿qué hay detrás del objeto?` para dirigir la pregunta a Qwen de forma
explícita, incluso si Gemini no está disponible. Una pregunta sobre la foto
durante `SafetyErrorAsk` no modifica ni reanuda la misión. «Borrar historial»
olvida también la foto seleccionada y sus preguntas anteriores.

# DetectSeafloorAnomalies

`DetectSeafloorAnomalies` acumula los barridos de
`/<robot_namespace>/sensors/multibeam/points` de un robot de superficie en un
mapa interno de celdas XY en `world_ned`. Compara la altura media de cada celda
con las celdas próximas y agrupa las que difieren al menos
`height_threshold` metros. Publica un `geometry_msgs/PointStamped` por cada
grupo nuevo en `/<robot_namespace>/actions/seafloor_anomalies`, con marco
`world_ned`. Sin anomalías no publica nada. Permanece en `RUNNING` hasta que
el árbol la detenga; si no llegan barridos transformables durante
`scan_timeout`, devuelve `FAILURE`. El mapa no se publica y se descarta al
detener la hoja.

```xml
<DetectSeafloorAnomalies robot_namespace="blueboat" height_threshold="0.5"
                         min_cluster_points="3" map_resolution="0.25"
                         neighborhood_radius="1.0" max_map_cells="10000"
                         scan_timeout="3.0"/>
```

Los topics de entrada y salida se pueden ajustar con `points_topic` y
`output_topic`, relativos al espacio de nombres del robot. El multibeam
publica sus puntos en `blueboat/multibeam_link`; esta hoja necesita la
transformación a `world_ned` para emitir coordenadas útiles en el mapa.
`map_resolution` determina el tamaño de cada celda en metros;
`neighborhood_radius` determina el entorno que se compara;
`min_cluster_points` exige ese número de celdas contiguas anómalas;
`max_map_cells` limita la memoria y expulsa las celdas menos recientes.
En una `Sequence`, esta hoja mantiene la secuencia en `RUNNING`; para mapear
mientras avanza la misión, colócala en una rama paralela que pueda detenerla.

# AvoidObstacle

`AvoidObstacle` es una hoja de misión independiente del árbol de seguridad.
Cuando `Navigation/FrontObstacle` indica `Front obstacle critically close`, solicita
`BODY_VELOCITY` y desplaza lateralmente al robot submarino hasta que el diagnóstico
llega a `OK`. Si el frente ya está en `OK`, termina sin mover el robot. Requiere
que el perfil confirme el eje `sway`. Un diagnóstico ausente, obsoleto o inválido
produce un fallo y detiene la maniobra.

```xml
<AvoidObstacle robot_namespace="bluegros" side="right" lateral_speed="0.2"
               message="{avoid_message}"/>
```

`side` admite `left` y `right` (predeterminado). El robot se desliza sin ordenar
avance ni giro hasta que el diagnóstico indique que el frente está despejado.
Si la hoja se interrumpe o el diagnóstico deja de ser válido, publica velocidad
cero; la intención de control caduca en el arbitrador. `SafetyErrorAsk` no cambia:
si ha pausado la misión, el operador debe resolverla y reanudarla antes de que
la misión ejecute `AvoidObstacle`.

# OrbitPointAction

`OrbitPointAction` ejecuta una vuelta alrededor de un punto con un robot submarino.
Configura y activa `/orbit_point_lifecycle_action_node`, envía la acción a
`/<robot_namespace>/actions/orbit_point` y cancela el objetivo si el árbol se detiene.
El servidor selecciona el modo de control, libera la intención y se desactiva al terminar.
Necesita un robot submarino con control de velocidad lateral y el servidor `OrbitPoint`
lanzado mediante `sura_actions`.

```xml
<OrbitPointAction robot_namespace="cirtesub" center_x="2.0" center_y="1.0"
                  center_z="-1.5" radius="2.0" message="{orbit_message}"/>
```

Para ejecutar la órbita editada en RViz, usa
`<OrbitPointAction robot_namespace="cirtesub" use_planned_orbit="true"/>`.
El puerto `timeout` es el límite de espera del BT (310 s por defecto); el tiempo
máximo del servidor se configura en `sura_actions/config/orbit_point.yaml`.

# Mission robot profiles

When `bt_runner` starts, it writes `tools/mission_generator/generated/mission_robot_profiles.json` for the
robots selected by `robot_namespace` or discovered from diagnostics. The JSON contains
robot name, sensor names, non-thruster actuator names, non-broadcaster controllers,
family and `available_axes`. Axes are derived from the body-force allocation matrix
and each thruster's calibrated force range. Axes below 10% of the strongest axis in
the same force/torque group, or below 1 N / 0.1 Nm, are omitted. Set the ROS parameter
`available_axis_relative_threshold` to change the relative cutoff.

Each profile also has `sensor_topics`, keyed by sensor name. Values are lists
because a sensor such as the DVL can publish several outputs. `bt_runner` reads
these from the active `ros2_control` parameters file selected in
`bringup_description.yaml` and uses `imu.raw_imu_topic` there when the IMU
broadcaster relies on a default name. An empty list means no output topic is
confirmed. The existing `sensors` list remains available to older consumers.

Camera names and image topics are configured in
`tools/mission_generator/robot_cameras.json`. The Mission Generator overlays those
lists onto the active robot profiles, so the configuration is stable even when
`bt_runner` regenerates its profile file. For example:

```json
"cameras": [
  {"name": "down camera", "topic": "/bluerov/image/down"}
]
```

Use the actual image topic published by the robot. The Mission Generator reads
this list when choosing a camera for `TakePhoto`. `bt_runner` also preserves
camera lists edited directly in the generated profile, but entries in the fixed
camera file take precedence. The `front_camera` and `right_camera` entries are
currently disabled in `bluerov_description/config/bringup_description.yaml`;
enable them there before requesting captures from those cameras.
The ZED image topic in the fixed file follows the older ZED ROS 2 wrapper
convention and is inferred from the point-cloud namespace used in this workspace.
Confirm it against `ros2 topic list` on the robot if the ZED wrapper version differs.

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

Mission Assistant receives `/sura_bt_runner/mission_status`, a transient-local
JSON snapshot with mission state and recent node transitions. It announces
mission starts, periodic progress, pauses, resumes and outcomes in the chat.
Routine action events stay out of the chat; progress and the final message
summarize meaningful movements in plain language.
When an action reports a reason for failure, `mission_status.message` and
`mission_status.event.message` provide one concise explanation naming the
failed step and its cause. The runner keeps the action's own detail in
`mission_status.failure.reason` and the most recent ROS error log produced by
that action in `mission_status.failure.terminal_error`. It does not publish a
separate action-failed chat event before the final failure. The chat can use
these fields to explain the outcome in one natural sentence instead of
repeating status updates. `TakePhoto` reports a missing camera topic, an image
timeout, or a file save error this way.
Critical safety diagnostics are published separately in `mission_status.safety`;
the chat identifies these as automatic safety stops rather than operator
cancellations.
Read-only questions about mission context, sensors and ROS topics use the same
chat without changing XML or resolving a pending safety decision. Topic samples
are bounded before display; an ambiguous sensor prompts a choice of output.
