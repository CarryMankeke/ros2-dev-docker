"""pseudo
# Nodo de teleoperación manual: lee sensor_msgs/Joy y emite comandos para base, brazo y gripper.
# Seguridad: deadman switch, watchdog de tiempo, preset Home/Sleep, QoS confiable y use_sim_time opcional.

procedimiento main():
  crear:
    - inicializar rclpy, nodo "joy_teleop"
    - declarar parámetros: axis_map_base, axis_map_arm, botones_gripper, modos (base/brazo/híbrido)
    - crear publishers: /cmd_vel (Twist), /arm_controller/joint_trajectory (JointTrajectory), /gripper/command (String)
    - suscribirse a /joy con QoS sensor_data (ver https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Quality-of-Service.html)
    - configurar timer_watchdog para enviar stop si no hay joystick en 0.5 s
  leer:
    - callback_joy(msg): interpreta ejes y botones
    - detectar cambio de modo por botones L1/R1; mantener estado modo_control
    - mapear sticks: base -> v, w; brazo -> delta pose cartesiana vía IK (documentado) o velocidades por junta
    - gripper: botones abrir/cerrar y fuerza +/-
    - presets: START -> home, SELECT -> sleep
  actualizar:
    - publicar Twist cuando modo base activo; limitar velocidades por parámetros (usar saturación como recomienda navegación ROS 2)
    - publicar JointTrajectoryPoint con posiciones objetivo cuando modo brazo activo
    - publicar comandos de gripper siempre que botones sean pulsados
    - refrescar reloj interno y métricas de latencia (use_sim_time si proviene de simulador)
  borrar:
    - on_shutdown: enviar velocidad cero, detener controladores y cerrar rclpy.shutdown()

if __name__ == '__main__': llamar main()
"""
