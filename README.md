# ros2-sim-vnc

Contenedor Docker + workspace ROS 2 (jazzy) para simular el robot móvil omnidireccional **mm_base** con el brazo 6DOF **mm_arm** y teleoperación micro-ROS.

## Requisitos locales
- Docker 24+ y `docker compose`.
- macOS/Linux/Windows con CPU x86_64 o arm64 (la imagen base es multi-arquitectura; no se fuerza `platform` en docker-compose).

## Flujo rápido
1. Construye la imagen y levanta el contenedor:
   ```bash
   docker compose build
   docker compose up -d
   ```
2. Accede al contenedor y prepara el workspace:
   ```bash
   docker compose exec -T ros2-vnc bash -lc '
   source /opt/ros/jazzy/setup.bash
   cd /root/ros2_ws
   colcon build --symlink-install
   '
   ```
3. Lanza los modos desde la raíz del repo:
   ```bash
   docker compose exec -T ros2-vnc bash -lc '
   source /opt/ros/jazzy/setup.bash
   source /root/ros2_ws/install/setup.bash
   ros2 launch mm_bringup modes.launch.py
   '
   ```

## Estructura clave
- `ros2_ws/src/mm_base_description`: base omnidireccional (URDF/Xacro + ros2_control).
- `ros2_ws/src/mm_arm_description`: brazo 6DOF con control por posición.
- `ros2_ws/src/mm_bringup`: launch files, configuración de teleop (`joy_teleop.py`), configuración de controladores y mundos SDF.
- `ros2_ws/src/mm_moveit_config`: configuración MoveIt 2 alineada con los controladores de brazo y gripper.
- `ros2_ws/src/mm_navigation`: parámetros Nav2 (mapas/costmaps/BT) para la fase de autonomía.
- `ros2_ws/src/mm_bringup/models`: assets locales para ser referenciados como `model://`.
- `extras/esp32_funduino_joy`: firmware ESP32 para publicar `/joy` por micro-ROS.
- `Dockerfile/docker-compose/supervisord.conf`: entorno con noVNC + micro-ROS Agent.

## Lanzamientos sugeridos (en container)
- SIM + RViz (predeterminado): `ros2 launch mm_bringup modes.launch.py`
- SIM + RViz + teleop ESP32: `ros2 launch mm_bringup modes.launch.py input:=esp32 teleop_mode:=hybrid`
- RViz + GUI de estados articulares (sin sim): `ros2 launch mm_bringup modes.launch.py launch_sim:=false clock_mode:=real input:=gui`

### Argumentos útiles (primera entrega, solo simulación/teleop)
- Posición inicial: `base_x/base_y/base_yaw` y offsets del brazo `arm_x/arm_y/arm_z/...`.
- Verbosidad Gazebo: `gz_verbosity:=3`.
- Caché de modelos: `model_cache_dir:=/tmp/mm_bringup` (el world por defecto apunta ahí).
- Bridges ROS↔Gazebo: definidos en `config/bridge_params.yaml` (LIDAR, IMU, cámaras base/nav, cámara brazo).
- Nav2/MoveIt están desactivados por defecto (`launch_nav2:=false`, `launch_moveit:=false`); actívalos solo después de la primera entrega.


## Recursos útiles
- `info.txt`: resumen de comandos más usados.
- `ros2_ws/src/mm_bringup/worlds/warehouse.sdf`: mundo liviano (plano + cubo) que incluye el ensamblaje generado en `/tmp/mm_bringup/mm_assembly.sdf`.
- `ros2_ws/src/mm_bringup/scripts/joy_teleop.py` + `config/joy_teleop.yaml`: lógica del teleop ESP32/joystick.
- `docs/estructura_pseudocodigo.md`: esqueleto completo en pseudocódigo de nodos, launchfiles y flujo de arranque para la simulación.
