"""pseudo
# Launch centralizado que coordina modos base/brazo/solo visualización.
# Host soportados: macOS/Linux arm64, Windows 10 x64 con WSL2 + NVIDIA (use_sim_time sincronizado).
# CRUD: crear lanzamientos, leer argumentos, actualizar combinaciones, borrar recursos temporales.

procedimiento generar_descripcion_lanzamiento():
  crear:
    - declarar argumentos clock_mode (sim|real), launch_sim (true|false), launch_display (true|false)
    - declarar argumento input (gui|esp32|micro_ros) y teleop_mode (base|arm|hybrid)
    - localizar paquete mm_bringup y rutas de sim.launch.py + display.launch.py
  leer:
    - condiciones IfCondition que activan simulación y RViz según flags
    - comentar QoS: use_sim_time:= (clock_mode == "sim")
  actualizar:
    - incluir sim.launch.py si launch_sim==true
    - incluir display.launch.py si launch_display==true
    - pasar variables comunes (clock_mode, input, teleop_mode) a sublanzadores
  borrar:
    - al cerrar, detener nodos y limpiar logs con ros2 lifecycle o Ctrl+C

retornar LaunchDescription con bloques anteriores
"""
