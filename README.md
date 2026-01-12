# ros2-dev-docker

Autor: Camilo Soto Villegas  
Contacto: camilo.soto.v@usach.cl  
Proyecto: clean_v2  
Target: ROS 2 Jazzy + Gazebo Harmonic

`clean_v2/` es la base oficial y unica del proyecto. La v1 fue eliminada del repositorio.

## Quick Start (Docker)
```bash
docker compose up -d
docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 launch mm_bringup sim_mm.launch.py headless:=false"
```

Si necesitas compilar antes de lanzar:
```bash
docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && cd /home/ros/ros2_ws && colcon build --symlink-install"
```

## Estructura estable
- `clean_v2/README.md`: guia principal del workspace.
- `clean_v2/docs/`: contratos TF, controladores y sensores.
