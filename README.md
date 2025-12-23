# ros2-sim-vnc (pseudocódigo)

Este repositorio se deja como guía en pseudocódigo para levantar y operar un entorno ROS 2 Jazzy + Gazebo Harmonic con teleoperación. Cada bloque describe acciones tipo CRUD (Crear, Leer, Actualizar, Borrar) sobre contenedores, workspace y lanzamientos.

## 1. Crear (build + arranque de contenedores)
```pseudo
procedimiento crear_entorno()
  prerequisitos (ver https://docs.ros.org/en/jazzy/Installation.html para dependencias base):
    - docker>=24 + docker compose
    - host macOS/Linux arm64 (ej. M1) o Windows 10 x64 (Intel i3 + NVIDIA GTX 1060) con WSL2
    - en Windows: habilitar WSL2 + Docker Desktop, activar runtime nvidia si hay GPU; en macOS: usar Rosetta para deps amd64
  paso 1: docker compose build                # construye imagen con ROS 2 + noVNC (desktop-full recomendado en Jazzy)
  paso 2: docker compose up -d                # arranca ros2-vnc y micro-ros-agent
  paso 3: docker compose ps                   # verifica estado de los servicios
fin
```

## 2. Leer (consultar estado del workspace y servicios)
```pseudo
procedimiento leer_estado()
  paso 1: docker compose logs -f --tail=200 ros2-vnc         # supervisor + VNC
  paso 2: docker compose logs -f --tail=200 micro-ros-agent  # agente UDP 8888
  paso 3: docker compose exec -T ros2-vnc bash -lc "ls /root/ros2_ws/src"   # paquetes
  paso 4: navegar a http://localhost:8080 para ver noVNC
fin
```

## 3. Actualizar (reconstruir workspace y lanzar modos)
```pseudo
procedimiento actualizar_workspace()
  paso 1: docker compose exec -T ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"
  paso 2: docker compose exec -T ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch mm_bringup modes.launch.py"
  paso 3: variantes (usar `use_sim_time` coherente según https://docs.ros.org/en/jazzy/How-To-Guides/Clock-and-Rate.html):
    - input:=esp32 teleop_mode:=hybrid   # teleop micro-ROS + sim
    - launch_sim:=false clock_mode:=real input:=gui  # solo RViz + sliders
fin
```

## 4. Borrar (limpieza y reinicio)
```pseudo
procedimiento borrar_entorno()
  paso 1: docker compose down                       # detiene y elimina contenedores
  paso 2: docker compose build --no-cache           # reconstruye imagen fresca si hace falta
  paso 3: dentro del contenedor -> rm -rf build install log en /root/ros2_ws
fin
```

## 5. Mapa rápido de carpetas en modo pseudocódigo
- `ros2_ws/src/mm_base_description`: URDF/Xacro de base omnidireccional.
- `ros2_ws/src/mm_arm_description`: URDF/Xacro de brazo 6DOF y gripper.
- `ros2_ws/src/mm_bringup`: launchers, configs de teleop y mundos SDF para la simulación.
- `extras/esp32_funduino_joy`: firmware de joystick micro-ROS.
- `docs/estructura_pseudocodigo.md`: esqueleto completo del stack en pseudocódigo.

## 6. Advertencias de seguridad en teleop
- Aplica watchdog < 0.5 s sin `/joy` → comandos cero.
- Deadband configurable en ejes del joystick para evitar ruido.
- Saturar velocidades de base y límites articulares del brazo antes de publicar comandos.

## 7. Compatibilidad de hosts (arm64 vs x86_64)
- macOS arm64 (M1/M2): imagen multi-arquitectura ya contemplada; preferir `platform: linux/arm64` en docker compose.
- Windows 10 x64 (Intel i3 + NVIDIA 1060): usar Docker Desktop con WSL2, `platform: linux/amd64` y habilitar soporte GPU (nvidia-container-toolkit) para RViz/Gazebo acelerados.
- Si falta GPU dedicada, forzar `LIBGL_ALWAYS_SOFTWARE=1` y mantener RViz en modo software render.
- Ajustar RMW según compatibilidad: rmw_cyclonedds predeterminado en Jazzy, opcional Fast DDS documentado en https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-middleware.html.
