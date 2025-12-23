"""pseudo
# Lanzador de simulación Gazebo Harmonic para base diferencial + brazo 6DOF.
# Asume sensores simulados y publicadores de clock/TF siempre activos.

procedimiento generar_descripcion_lanzamiento_sim():
  crear:
    - argumentos: clock_mode (sim|real), input (gui|esp32|micro_ros), teleop_mode (base|arm|hybrid)
    - cargar plugins de Gazebo con world seleccionado (minimal.world.sdf o warehouse.sdf)
    - incluir nodos de control: diff_drive_controller, joint_trajectory_controller, gripper_controller (según ros2_control de Jazzy)
  leer:
    - archivos de configuración en config/*.yaml para controladores base/brazo/gripper
    - modelos Gazebo desde models/* con colisiones simplificadas
  actualizar:
    - spawn de mm_base y mm_arm en poses iniciales seguras
    - suscripción a /joy (input gui o micro-ROS) y publicación /cmd_vel + trayectorias articuladas
    - publicación constante de /tf y /joint_states con use_sim_time:=true (apoyarse en tutorial de tiempo simulado de Jazzy)
  borrar:
    - detener simulación, borrar entidades y liberar recursos de gazebo server/ client

retornar LaunchDescription con bloques de simulación
"""
