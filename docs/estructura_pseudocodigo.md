# Estructura en pseudocódigo para el stack ROS 2 (Jazzy)

Guía de esqueleto para el sistema de teleoperación y simulación del manipulador móvil (base omnidireccional + brazo 6DOF + gripper) en ROS 2 Jazzy. No es código ejecutable; organiza paquetes, nodos, tópicos y launchfiles conforme a la documentación oficial de ROS 2, Gazebo Harmonic, Nav2 y MoveIt 2.

## 0. Alcance de la primera entrega
- Control **100% manual**: base móvil + brazo 6DOF + gripper sin autonomía.
- **Todo en simulación** hasta completar el código, con publicación de datos (JointState/TF) en todo momento.
- **RViz siempre activo** para visualización del modelo URDF y el estado del robot.
- **Joystick/gamepad** como único dispositivo de entrada, con cambio de modo Base/Brazo y botones dedicados para el gripper y presets.

## 1. Paquetes y carpetas
- `mm_base_description/`: URDF/Xacro de la base, transmisión, plugins Gazebo/Ignition y controladores `ros2_control`.
- `mm_arm_description/`: URDF/Xacro del brazo 6DOF, límites articulares, transmisiones y frames `tool0`/`ee_link`.
- `mm_bringup/`: launchfiles, configuraciones de control (`ros2_control`), RViz, Gazebo y teleop (parámetros en YAML, sin hardcodeo).
- `mm_bringup/config/`: configuración MoveIt 2 (SRDF, OMPL, pipelines, scene monitor) alineada con los controladores de brazo.
- `mm_bringup/maps/`: mapas Nav2 y `mm_bringup/config/nav2_params.yaml` para navegación.
- `extras/esp32_funduino_joy/`: firmware micro-ROS que publica `sensor_msgs/msg/Joy` (opcional).
- `docs/`: material de diseño y guías (este archivo y anexos), siempre referenciado desde el README.

## 2. Arquitectura de nodos cooperativos
- **Nodo `joy` (driver de joystick)**: publica `/joy` (`sensor_msgs/msg/Joy`) con `SensorDataQoS`.
- **Nodo `teleop_bridge`**: suscrito a `/joy`; convierte ejes/botones en comandos para base, brazo y gripper con watchdog de inactividad (<0.5 s → comandos cero).
- **Controladores simulados**:
  - Base: consume `/cmd_vel` (`geometry_msgs/msg/Twist`) y aplica velocidades a las ruedas.
  - Brazo: controlador de trayectoria articular (`trajectory_msgs/msg/JointTrajectory`).
  - Gripper: posición o esfuerzo (`std_msgs/msg/Float64`).
- **Visualización**: `robot_state_publisher` (+ `joint_state_publisher` si no hay sim) y `rviz2` cargando el URDF.
- **Supervisión**: `diagnostic_updater`/`diagnostic_aggregator` para exponer estado de teleop, modo activo y latencias.

## 3. Modos y mapeo del joystick
- **Cambio de modo**: botón de hombro (p. ej. L1/R1) alterna entre `BASE` y `ARM`. Mientras está activo el botón → modo Base; al soltar → modo Brazo. Si se usa _toggle_, registrar flanco de subida y rebotado mínimo (debounce 50–100 ms) para evitar cambios espurios.
- **Base (modo BASE)**: `RB + stick izquierdo` controla `vx` (adelante/atrás) y `vy` (izq/der en holonómica); `RB + stick derecho` controla `wz` (giro). Escalas moderadas por defecto; opcional ajuste de ganancia con botones (BACK+Y/A/X/B) saturado en ±1.0 m/s y ±2.0 rad/s.
- **Brazo (modo ARM)**:
  - Stick izquierdo → traslaciones cartesianas `Δz` (arriba/abajo) y `Δx` (alcance).
  - Stick derecho → orientación del efector `Δroll`, `Δpitch`.
  - L1/R1 (en modo brazo) → `Δy` (profundidad del efector).
  - Aplicar _deadband_ configurable a los ejes para evitar ruido en la cinemática inversa (umbral típico 0.05–0.1).
  - Limitar incrementos para respetar velocidades máximas articulares y suavizar el solver IK.
  - Alternativa (no preferida): selección de articulación + control articular directo.
- **Gripper**: botones dedicados abrir/cerrar (p. ej. círculo/cuadrado). Opcional dos botones extra para aumentar/disminuir fuerza de agarre con límites documentados.
- **Presets**: `START` → pose Home; `SELECT` → pose Stow/Reposo. Se pueden usar en cualquier modo y deben cancelarse si el watchdog de joy expira.

## 4. Nodo `teleop_bridge` (pseudocódigo detallado)
```pseudo
node teleop_bridge
  subscribe /joy (sensor_msgs/msg/Joy)
  publish /cmd_vel (geometry_msgs/msg/Twist)
  publish /mm_arm/cartesian_cmd (geometry_msgs/msg/Twist|Pose)
  publish /mm_arm/joint_cmd (trajectory_msgs/msg/JointTrajectory)
  publish /mm_arm/gripper_cmd (std_msgs/msg/Float64)

  params:
    base_scale, arm_cartesian_scale, deadband
    mode_cycle_buttons, gripper_buttons, preset_buttons
    watchdog_timeout, qos_profiles

  state:
    mode in {BASE, ARM}
    prev_buttons = []
    prev_axes = []

  on_joy(msg):
    msg.axes <- apply_deadband(msg.axes, deadband)

    mode <- update_mode(msg.buttons, prev_buttons, mode_cycle_buttons)

    if mode == BASE:
      vx, vy, wz <- map_axes_to_base(msg.axes, base_scale)
      publish Twist(vx, vy, wz)                           # QoS SensorData

    if mode == ARM:
      dpose <- map_axes_to_ee(msg.axes, arm_cartesian_scale) # Δx, Δy, Δz, Δroll, Δpitch, (Δyaw opcional)
      pose_goal <- clamp_to_workspace(current_pose + dpose)
      q_target <- inverse_kinematics(pose_goal)
      if q_target is valid and within joint_limits:
        publish JointTrajectory(q_target)                  # header.stamp = now(); single point con time_from_start > 0

    handle_gripper(msg.buttons, prev_buttons, gripper_buttons)   # abrir/cerrar y fuerza opcional
    handle_presets(msg.buttons, prev_buttons, preset_buttons)    # home/stow

    prev_buttons <- msg.buttons
    prev_axes <- msg.axes
```

## 5. Topics, frames y publicación de estado
- **Frames**: `base_link`, `odom`, `arm_base_link`, `tool0` (TCP). Publicados por `robot_state_publisher` a partir del URDF.
- **Estados articulares**: `joint_state_broadcaster` (en sim) o `joint_state_publisher` (GUI) cuando no hay sim.
- **Comandos**:
  - Base: `/cmd_vel` (`Twist`).
  - Brazo: `/mm_arm/joint_cmd` (`JointTrajectory`) o comandos cartesianos intermedios `/mm_arm/cartesian_cmd`.
  - Gripper: `/mm_arm/gripper_cmd` (`Float64` posición/esfuerzo).
- **Diagnóstico**: `/diagnostics` agrega latencia de joy, modo activo y estado del watchdog.

## 6. Launchfiles (versión buenas prácticas 2025)

### `display.launch.py` (URDF + RViz)
```pseudo
include xacro -> robot_description
start robot_state_publisher
start joint_state_publisher_gui (opcional)
start rviz2 with mm_display.rviz
```

### `sim.launch.py` (Gazebo + controladores)
```pseudo
args:
  use_sim_time:=true
  world:=minimal.world.sdf
  base_prefix, arm_prefix
  base_scale, arm_scale
  base_pose:={x,y,z,yaw}
  arm_pose:={x,y,z,roll,pitch,yaw}
  gz_verbosity:=4
  launch_gz:=true|false
  model_cache_dir:=/tmp/mm_bringup

set_env GZ_SIM_RESOURCE_PATH:=<pkg_share>/models
include gz_sim.launch.py with world

generate URDF/SDF:
  xacro base (controllers_file:=config/base_controllers.yaml)
  xacro arm (controllers_file:=config/arm_controllers.yaml)
  assembly.sdf (incluye base en base_pose y brazo en arm_pose)

spawn robot in Gazebo:
  include assembly.sdf

TF estático base->arm con arm_pose

load/start controllers via ros2_control (encadenados con eventos):
  joint_state_broadcaster
  mecanum_drive_controller
  arm_trajectory_controller
  gripper_controller (si aplica)

bridges (parametrizados desde YAML):
  clock_bridge: /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
  base_sensors: scan, camera, imu (QoS explícito)
  arm_sensors: camera, camera_info, imu

 wait_for /clock y servicios de controller_manager con timeout claro
```

### `teleop.launch.py`
```pseudo
args: input:={joy, esp32}, teleop_mode:={base, arm, hybrid}
start joy node (or micro-ROS agent subscriber)
start teleop_bridge with params from config/joy_teleop.yaml
```

### `modes.launch.py` (entrypoint unificado)
```pseudo
args:
  launch_sim:=true|false
  clock_mode:=sim|real
  input:=joy|esp32|gui
  teleop_mode:=hybrid
  world:=minimal.world.sdf
  base_pose:=...
  arm_pose:=...

if launch_sim:
  include sim.launch.py (pasar world, base_pose, arm_pose)
else:
  start joint_state_publisher_gui

include display.launch.py (rviz siempre activo)
if input in {joy, esp32}: include teleop.launch.py
```

## 7. Configuración sugerida `config/joy_teleop.yaml`
```yaml
teleop_bridge:
  ros__parameters:
    mode_cycle_buttons: [L1, R1]
    base_scale: {vx: 0.6, vy: 0.6, wz: 1.2}
    arm_cartesian_scale: {dx: 0.01, dy: 0.01, dz: 0.01, droll: 0.05, dpitch: 0.05, dyaw: 0.05}
    gripper:
      open_button: circle
      close_button: square
      force_increment_buttons: [triangle, cross]
    presets:
      home_button: start
      stow_button: select
    watchdog_timeout: 0.5   # segundos sin /joy -> comandos cero
    qos_profiles:
      joy: sensor_data
      cmd_vel: sensor_data
      arm_cmd: reliable_depth5
```

## 8. Simulación y visualización (siempre activa)
- **Controladores en modo sim**: los plugins/controladores de base, brazo y gripper responden a los tópicos de comando sin hardware real.
- **Publicación de estados**: `joint_state_broadcaster` y `robot_state_publisher` mantienen TF y `JointState` actualizados en cada ciclo.
- **RViz siempre corriendo**: carga el URDF y refleja movimientos de base/brazo/gripper en tiempo real.
- **Verificación**: mover el joystick y validar que `/cmd_vel`, `/mm_arm/joint_cmd` y `/mm_arm/gripper_cmd` cambian; observar en RViz y en los tópicos para depurar.

## 9. Flujo de arranque (host → contenedor → ROS 2)
```pseudo
host$ docker compose build && docker compose up -d
host$ docker compose exec -T ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"
host$ docker compose exec -T ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 launch mm_bringup modes.launch.py"

# Modos más usados:
# ros2 launch mm_bringup modes.launch.py gz_verbosity:=3 base_x:=0 base_y:=0 base_yaw:=0.0
# ros2 launch mm_bringup modes.launch.py launch_sim:=false clock_mode:=real input:=gui
```

## 10. Checklist de integración
- [ ] URDF/Xacro cargan sin warnings (`ros2 launch mm_bringup display.launch.py`).
- [ ] Controllers activos y publicando TF/JointState.
- [ ] Teleop mueve base y brazo en sim; gripper responde.
- [ ] RViz muestra el robot en todo momento.
- [ ] Gazebo simula sensores y publica tópicos requeridos.
- [ ] micro-ROS agent conectado cuando se usa ESP32.
- [ ] Watchdog corta movimientos tras perder `/joy`.
- [ ] QoS documentada coincide con configuración en YAML.

## 11. Buenas prácticas (Jazzy + Gazebo Harmonic + Nav2 + MoveIt 2)
- **ros2_control en Harmonic**: cargar primero `joint_state_broadcaster`, luego controladores de la base y el brazo con `spawner.py` desde Gazebo; fijar `use_sim_time:=true` y comprobar que la `controller_manager` responde antes de enviar comandos.
- **Sincronía reloj**: propagar `/clock` desde Gazebo y usar `rclcpp::Clock(RCL_ROS_TIME)` en nodos de control. RViz y MoveIt 2 deben lanzar con `use_sim_time` habilitado.
- **QoS y confiabilidad**: usar `SensorDataQoS` para `/joy` y `/cmd_vel` (baja latencia); emplear `Reliable` + `Depth>1` para comandos del brazo (`JointTrajectory`) y TF estático.
- **Nav2 (fase posterior)**: mantener mapas en `mm_bringup/maps/` y parámetros en `mm_bringup/config/nav2_params.yaml`. La base debe exponer `/cmd_vel` y `/tf/odom` coherentes; usar el `controller_server` tipo `dwb` u otro que respete límites de velocidad del teleop para pruebas manuales.
- **MoveIt 2**: generar `mm_arm.srdf` con `moveit2_setup_assistant`; alinear nombres de grupos con los controladores `ros2_control` (`arm`, `gripper`). Habilitar `planning_scene_monitor` en sim para consumir TF/JointState.
- **Seguridad en teleop**: aplicar `deadband`, saturación de velocidades y watchdog (p. ej., _timeout_ < 0.5 s sin `/joy` → publicar Twist cero). Para el brazo, validar soluciones IK dentro de límites articulares y respetar `collision_check` cuando MoveIt 2 esté activo.
- **Layouts configurables**: centralizar mapeos en YAML y documentar equivalencias por gamepad (Xbox/DS4) para evitar _hardcode_.
- **Logs y diagnóstico**: usar `diagnostic_updater` para exponer estado de joystick, modo activo y latencias de comando; activar `ros2 topic hz` en validaciones.
- **Calidad de datos**: registrar _bag_ en pruebas críticas con `/joy`, `/cmd_vel`, `/mm_arm/joint_cmd`, `/tf` y `/clock` para reproducibilidad.
- **Cancelación segura**: permitir `Ctrl+C` o servicio de parada que publique comandos cero y desactive controladores si el watchdog persiste.
- **Parametrización de poses**: no fijar `base_x/base_y` ni offsets en SDF; pasar `base_pose` y `arm_pose` como argumentos de launch y propagar a los includes de Gazebo/TF para soportar múltiples instancias y CI determinista.
- **Puentes ROS↔GZ configurables**: describir mappings en `config/bridge_params.yaml` y cargar desde launch, evitando cadenas embebidas. Documentar QoS y tipos exactos por tópico.
- **Rutas temporales controladas**: generar artefactos (URDF/SDF) en una carpeta cache del workspace (`ros2_ws/tmp` o `~/.cache/mm_bringup`) en lugar de `/tmp`, y limpiar bajo demanda.
- **Integración MoveIt/Nav2**: mantener configuración de planificación y navegación en archivos dedicados (`config/moveit_*.yaml`, `config/nav2_params.yaml`, `behavior_trees/`), con lanzadores propios e incluidos desde `modes.launch.py`. Ver `docs/arquitectura_moveit_nav2.md` para guía detallada.
- **Cache de modelos parametrizable**: usar `model_cache_dir` como argumento de launch para los URDF/SDF generados y alinear la ruta con los mundos SDF que incluyan el ensamblaje.
- **Sensores documentados**: si hay múltiples cámaras (frontal y navegación), reflejarlas en URDF y bridge YAML (`/mm_base/camera`, `/mm_base/nav_camera`) y definir QoS acorde (best_effort aceptable en sim).
- **BT Nav2 predeterminada**: usar el árbol `navigate_w_replanning_and_recovery.xml` de `nav2_bt_navigator` como valor por defecto (`nav2_bt`) para navegación básica, y versionarlo si se personaliza.
- **Colisiones en MoveIt**: cargar `config/moveit_planning_scene.yaml` para tener monitor de escena con TF/JointState y ejecución de trayectorias con tolerancias, asegurando que la planificación considere colisiones.
- **Gripper sin DOF**: por ahora no se define controlador de gripper en ros2_control ni MoveIt; cuando se añada un actuador, crear la junta en URDF y extender los YAML correspondientes antes de activar su spawner.
- **JointState global fusionado**: publicar `/joint_states` a partir de `/mm_base/joint_states` y `/mm_arm/joint_states` con un agregador (e.g., `joint_state_aggregator.py`) para que herramientas genéricas (RViz/MoveIt) consuman un único flujo manteniendo los namespaces internos.
- **Docker multi-arquitectura**: la imagen base Jazzy es multi-arch (amd64/arm64) y `docker-compose.yml` no fuerza `platform`, permitiendo host macOS/Windows/Linux sin emulación manual.
