"""pseudo
# Nodo utilitario que publica URDF combinado de base + brazo para RViz y simulación.
# Incluye selector de prefijo y scale para reutilizar en múltiples instancias.

procedimiento main():
  crear:
    - inicializar rclpy, nodo "robot_description_publisher"
    - parámetros: prefix (mm_), scale (1.0), use_sim_time (bool), controllers_file (ruta YAML)
    - leer xacro mm_base.urdf.xacro y mm_arm.urdf.xacro, concatenar en <robot_description> (usar xacro conforme https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro.html)
    - publisher de std_msgs/String en /robot_description con QoS reliable
  leer:
    - validar existencia de archivos xacro y controllers YAML; registrar advertencias si faltan
    - exponer parámetros vía servicios de rcl_interfaces/GetParameters
  actualizar:
    - reconstruir descripción cuando cambien parámetros o se reciba señal de recarga
    - mantener publicación latched para nuevos suscriptores (RViz, robot_state_publisher)
  borrar:
    - detener nodo y limpiar timers/hilos

if __name__ == '__main__': llamar main()
"""
