# MoveIt 2 base para mm_moveit_config
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Objetivo
1. Habilitar planeacion y ejecucion para el brazo y el gripper con MoveIt 2.
2. Usar controladores de trayectoria con interfaz de posicion para mantener compatibilidad con MoveIt.

## Pasos
1. Compilar el workspace:
   - `cd clean_v2/ros2_ws && colcon build --symlink-install`
2. Lanzar simulacion del manipulador:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_`
3. Lanzar MoveIt (solo move_group):
   - `source install/setup.bash`
   - `ros2 launch mm_moveit_config moveit.launch.py namespace:=mm1 prefix:=mm1_`

## Notas
- La configuracion usa `JointTrajectoryController` con interfaz de posicion, alineado con el flujo recomendado por MoveIt 2 para ejecutar trayectorias.
- Si necesitas multiples robots, ejecuta un `move_group` por namespace y usa prefijos distintos.

## Seguridad
- Antes de habilitar movimiento real, agrega un watchdog de `cmd_vel` y una parada de emergencia fisica o virtual.
- Valida limites y aceleraciones del brazo para evitar golpes cuando el controlador recibe trayectorias.

## Referencias
- MoveIt 2 (Jazzy): https://moveit.picknik.ai/main/doc/ros2/index.html
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/index.html

## Guia especifica para mm1 (Docker + VNC)
- `docs/moveit_setup_mm1.md`



# Smoke
docker compose exec ros2-vnc bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /home/ros/ros2_ws/install/setup.bash && \
 ros2 run mm_bringup run_smoke_tests.sh"
