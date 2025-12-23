# Estructura en pseudocódigo del control teleoperado 6DOF

"""pseudo
arquitectura_nodos():
  joystick_node:
    rol: leer gamepad y publicar sensor_msgs/Joy en /joy
    watchdog: si no hay mensajes en <0.5 s -> publicar cero (usar QoS sensor_data como sugiere Jazzy)
  teleop_node:
    suscribe: /joy
    publica: /cmd_vel (base), /arm_command (FollowJointTrajectory), /gripper_command
    modo_control: BASE <-> BRAZO con botones de hombro (L1/R1)
    presets: START -> home_pose ; SELECT -> pose_reposo
    gripper: boton abrir -> abrir; boton cerrar -> cerrar; botones fuerza +/- opcionales
  controladores_simulados:
    base: aplica geometry_msgs/Twist a ruedas (diferencial u omnidireccional)
    brazo: resuelve cinemática inversa para 6DOF y envía trayectorias (ros2_control según https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-ros2-control.html)
    gripper: acepta posición o esfuerzo objetivo
  visualizacion_rviz:
    lanza RViz siempre activo con URDF + TF + JointState
    entrada: joint_state_publisher y robot_state_publisher desde simulación
  clock_sim:
    usar use_sim_time=true y fuente /clock desde Gazebo para sincronía

mapeo_joystick():
  modo_base():
    stick_izq.y -> v_lineal_x ; stick_izq.x -> v_lineal_y (si base holonómica)
    stick_der.x -> w_angular_z
    botones velocidad: BACK+{A/B/X/Y} ajustan escala lineal/ang
  modo_brazo():
    stick_izq: delta_z (vertical) y delta_x (alcance)
    stick_der: delta_pitch (vertical) y delta_roll (horizontal)
    L1/R1: delta_y (profundidad efector)
    enviar objetivos cartesianos -> IK -> posiciones articulares -> publicar trayectoria
  gripper():
    boton_open -> posición 0%
    boton_close -> posición 100%
    fuerza+: incrementar esfuerzo; fuerza-: disminuir esfuerzo
  presets():
    START -> home seguro ; SELECT -> plegar

lanzadores_simulacion():
  launch_sim():
    - arrancar Gazebo con mundo base y plugins de control
    - spawn URDF del manipulador móvil (usar spawn_entity de ros_gz)
    - iniciar joint_state_publisher + robot_state_publisher (mantener use_sim_time)
    - lanzar teleop_node y joystick_node
    - abrir RViz con vista del robot
  launch_rviz_solo():
    - omitir Gazebo, solo URDF + joint_state_publisher_gui
    - usar clock real y RViz para inspección de geometría

compatibilidad_host():
  macOS arm64 (M1/M2): platform linux/arm64, render software si no hay GPU dedicada
  Windows 10 x64 (Intel i3 + NVIDIA 1060): Docker Desktop + WSL2, platform linux/amd64, nvidia-container-runtime para RViz/Gazebo
  fallback: LIBGL_ALWAYS_SOFTWARE=1 si no se expone GPU
"""

Notas:
- Mantener límites articulares y saturación de velocidad antes de publicar comandos.
- Documentar en launch files QoS explícito para `/joy`, `/cmd_vel` y `/arm_command` (reliable, depth=10) por seguridad.
- Todos los comandos se ejecutan en simulación hasta completar hardware; aprovechar `/clock` para sincronizar RViz y controladores (ver https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Simulators/Simulation-Time.html).
