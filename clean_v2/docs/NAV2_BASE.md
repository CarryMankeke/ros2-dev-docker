# Nav2 base para mm_bringup
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Objetivo
1. Preparar una configuracion base de Nav2 para una base omni.
2. Mantener sincronizacion con `/clock` y `use_sim_time`.

## Pasos
1. Compilar el workspace:
   - `cd clean_v2/ros2_ws && colcon build --symlink-install`
2. Lanzar simulacion con LiDAR:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_ enable_lidar:=true`
3. Lanzar Nav2 (SLAM por defecto):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup nav2_min.launch.py namespace:=mm1 prefix:=mm1_ slam:=true`

## Notas
- El LiDAR publica en `/<namespace>/scan` y se puentea con `ros_gz_bridge`.
- Se configura QoS explicito para `/scan` (best_effort, volatile, keep_last, depth 10) en los costmaps para evitar bloqueos por datos de sensores.
- Ajusta velocidades y limites en `nav2_params.yaml.in` segun dinamica real de la base.

## Seguridad
- Agrega parada de emergencia y watchdog de `cmd_vel` antes de operar el robot real.
- Limita velocidades maximas en Nav2 y en el controlador base para evitar movimientos inesperados.

## Referencias
- Nav2: https://navigation.ros.org/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/index.html
