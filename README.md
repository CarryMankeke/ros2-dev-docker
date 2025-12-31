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
   cd /home/ros/ros2_ws
   colcon build --symlink-install
   '
   ```
3. Lanza los modos desde la raíz del repo:
   ```bash
   docker compose exec -T ros2-vnc bash -lc '
   source /opt/ros/jazzy/setup.bash
   source /home/ros/ros2_ws/install/setup.bash
   ros2 launch mm_bringup modes.launch.py
   '
   ```

## Estructura clave
- `ros2_ws/src/mm_base_description`: base omnidireccional (URDF/Xacro + ros2_control).
- `ros2_ws/src/mm_arm_description`: brazo 6DOF con control por posición.
- `ros2_ws/src/mm_bringup`: launch files, configuración de teleop (`joy_teleop.py`), configuración de controladores y mundos SDF.
- `ros2_ws/src/mm_bringup/config`: configuración MoveIt 2 (SRDF + planning + controllers) alineada con el brazo.
- `ros2_ws/src/mm_bringup/maps` + `ros2_ws/src/mm_bringup/config/nav2_params.yaml`: mapas y parámetros Nav2.
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
- Depuración Gazebo GUI: `launch_gz:=false` y lanzar `gz sim` por separado si el GUI se cierra.
- Bridges ROS↔Gazebo: definidos en `config/bridge_params.yaml` (LIDAR, IMU, cámaras base/nav, cámara brazo).
- Nav2 está desactivado por defecto (`launch_nav2:=false`); MoveIt está activo en sim.


## Documentación y análisis

### 📚 Guías principales
- `info.txt`: resumen de comandos más usados.
- `CONTRIBUTING.md`: guía para colaboradores (flujo de trabajo, estilo de código, convenciones).
- `CHANGELOG.md`: historial de cambios (sigue [Keep a Changelog](https://keepachangelog.com/)).

### 🔍 Análisis de estado (27-30 de diciembre de 2025)
- **`RESUMEN_EJECUTIVO.md`**: Panorama general del proyecto, problemas encontrados y recomendaciones.
- **`PROBLEMAS_ACTUALES.md`**: Listado detallado de 3 problemas críticos, 5 mayores y 5 menores con impacto.
- **`MAPA_PROYECTO.md`**: Arquitectura visual, flujo de datos, estructura de ficheros y ciclo de arranque.
- **`PLAN_ACCION_INMEDIATO.md`**: Soluciones paso a paso para resolver P1 (SRDF dinámico), P2 (Controllers dinámicos), P3 (Validación rutas).
- **`AUDIT_REPORT.md`**: auditoría exhaustiva del proyecto (problemas críticos, mayores, menores, roadmap).

### 📖 Documentación técnica
- `ros2_ws/src/mm_bringup/worlds/minimal.world.sdf`: mundo liviano (plano + cubo) que incluye el ensamblaje generado en `/tmp/mm_bringup/mm_assembly.sdf`.
- `ros2_ws/src/mm_bringup/scripts/joy_teleop.py` + `config/joy_teleop.yaml`: lógica del teleop ESP32/joystick.
- `docs/estructura_pseudocodigo.md`: esqueleto completo en pseudocódigo de nodos, launchfiles y flujo de arranque para la simulación.
- `docs/arquitectura_moveit_nav2.md`: arquitectura de MoveIt 2 y Nav2 para fases futuras.

## CI/CD y Testing

Este proyecto incluye:
- **GitHub Actions** (`.github/workflows/ci.yaml`): `colcon build`, linters, smoke tests en cada push/PR a `main` y `develop`.
- **Local testing**: ejecuta `colcon test` en Docker después de `colcon build`.

Para validar cambios localmente:
```bash
docker compose exec -T ros2-vnc bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /home/ros/ros2_ws
  colcon build --symlink-install && colcon test
'
```
