# MoveIt Setup Assistant para mm1 (Docker + VNC)
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2
Target: ROS 2 Jazzy + Gazebo Harmonic

## Requisitos previos en Docker
1) Levantar contenedor (si no esta arriba no existe ROS dentro):
   - `docker compose up -d`
2) Source de ROS (obligatorio para que exista el comando `ros2`):
   - `docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 --help"`
3) Verificar MoveIt instalado:
   - `docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep moveit"`
4) Nota importante sobre moveit-resources:
   - `moveit_resources_panda_moveit_config` es solo un sanity check opcional.
   - No 'tenerlo' no significa que MoveIt falle con mm1.
5) Elegir comando correcto:
   - Usa `docker compose exec ros2-vnc ...` (recomendado).
   - `docker exec ros2-vnc ...` falla porque el contenedor real es `ros2_vnc_container`.

## Por que el Setup Assistant NO debe cargar xacro directamente
- El Setup Assistant espera un URDF ya expandido.
- Un archivo `.xacro` contiene macros y puede requerir argumentos.
- Si se intenta cargar `.xacro` directo, el parser falla y en VNC suele cerrarse sin mensaje.
- Esto es mas comun en Jazzy + VNC + macOS M1 por el stack grafico.

## Procedimiento correcto y estable para mm1
Nota: el Setup Assistant NO debe ejecutarse desde el host. SIEMPRE debe lanzarse dentro del contenedor
con `docker compose exec`, ya que el GUI en macOS M1 depende del entorno grafico del contenedor.

1) Expandir xacro a URDF:
   - `docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run xacro xacro /home/ros/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro > /tmp/mm1.urdf"`
2) Validar URDF (recomendado):
   - `docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && check_urdf /tmp/mm1.urdf"`
3) Lanzar Setup Assistant:
   - `docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run moveit_setup_assistant moveit_setup_assistant"`
4) En el GUI, cargar el URDF generado:
   - `/tmp/mm1.urdf`
5) Si el GUI se cierra, relanzar y volver a cargar el URDF ya expandido.

## Diagnostico de cierres del Setup Assistant
1) Verificar si el proceso sigue vivo:
   - `docker compose exec ros2-vnc bash -lc "ps aux | grep moveit_setup_assistant | grep -v grep"`
2) Si no hay proceso, la causa mas probable es parsing fallido del URDF o fallo grafico de VNC.
3) Evitar recargas rapidas y nunca abrir un `.xacro` directo.

## Nota especifica para macOS M1 + Docker + VNC
- El renderizado en VNC es fragil en Apple Silicon.
- Evita recargar varias veces el mismo archivo en pocos segundos.
- No ejecutes el Setup Assistant fuera del contenedor.
- Siempre usa URDF expandido para evitar cierres silenciosos.
