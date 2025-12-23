"""pseudo
# Lanzador de RViz + descripción URDF para teleoperación 6DOF.
# Compatibilidad: macOS arm64 con software rendering o Windows x64 con GPU NVIDIA (WSL2 + mesa/vgpu).

procedimiento generar_descripcion_lanzamiento_display():
  crear:
    - declarar argumentos: clock_mode (sim|real), input (gui|esp32|micro_ros), teleop_mode (base|arm|hybrid)
    - configurar use_sim_time := (clock_mode == "sim")
    - preparar Node robot_state_publisher con URDF mm_base + mm_arm (xacro) y QoS confiable
  leer:
    - archivos URDF desde mm_base_description/urdf y mm_arm_description/urdf
    - layout RViz mm_display.rviz con vistas de TF, JointState y markers de metas
  actualizar:
    - lanzar RViz con argumento --display-config mm_display.rviz
    - cargar joint_state_publisher_gui cuando input=="gui" para emulación de sliders
  borrar:
    - cerrar RViz y detener publishers de estado al apagar la sesión

retornar LaunchDescription con nodos display
"""
