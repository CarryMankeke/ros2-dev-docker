# MAPA DEL PROYECTO - ros2-sim-vnc

## 🏗️ ARQUITECTURA GENERAL

```
┌─────────────────────────────────────────────────────────────────┐
│                        HOST (macOS/Linux)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  docker compose build && docker compose up -d                  │
│                                                                  │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│              Docker Contenedor (Linux x86_64/arm64)             │
│                  ros:jazzy-ros-base-noble                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  supervisord.conf (entrypoint)                          │   │
│  │  ├─ [program:xvfb] → Xvfb :0 (framebuffer virtual)    │   │
│  │  ├─ [program:x11vnc] → x11vnc -forever -usepw          │   │
│  │  ├─ [program:novnc] → noVNC (websocket → :8080)        │   │
│  │  ├─ [program:xfce4] → XFCE4 desktop                    │   │
│  │  └─ [program:ros2_humble] → bash (esperando comandos)  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  ROS 2 Jazzy Workspace (/home/ros/ros2_ws)            │   │
│  │                                                          │   │
│  │  src/                                                    │   │
│  │  ├─ mm_base_description/       (base omnidireccional)  │   │
│  │  │  └─ urdf/mm_base.urdf.xacro                          │   │
│  │  │     - Propiedades: scale, prefix                     │   │
│  │  │     - 4 ruedas mecanum (omnidireccionales)           │   │
│  │  │     - Plugins: lidar, cámara, IMU                    │   │
│  │  │                                                      │   │
│  │  ├─ mm_arm_description/        (brazo 6DOF)           │   │
│  │  │  └─ urdf/mm_arm.urdf.xacro                           │   │
│  │  │     - 6 joints rotacionales                          │   │
│  │  │     - Gripper (no modelado en xacro)                 │   │
│  │  │     - EndEffector frame: tool0                       │   │
│  │  │                                                      │   │
│  │  └─ mm_bringup/                (punto de entrada)      │   │
│  │     ├─ launch/                                          │   │
│  │     │  ├─ modes.launch.py       (ENTRYPOINT)            │   │
│  │     │  ├─ sim.launch.py         (Gazebo)               │   │
│  │     │  ├─ display.launch.py     (RViz sin sim)         │   │
│  │     │  ├─ moveit.launch.py      (MoveIt 2)             │   │
│  │     │  ├─ nav2.launch.py        (Navegación)           │   │
│  │     │  └─ teleop.launch.py      (Teleoperación)        │   │
│  │     ├─ config/                                          │   │
│  │     │  ├─ bridge_params.yaml    (ROS↔Gazebo)           │   │
│  │     │  ├─ base_controllers.yaml ❌ HARDCODEADO          │   │
│  │     │  ├─ arm_controllers.yaml  ✅ OK                  │   │
│  │     │  ├─ joy_teleop.yaml       ❌ HARDCODEADO          │   │
│  │     │  ├─ moveit_servo.yaml     ✅ OK                  │   │
│  │     │  ├─ mm_arm.srdf           ❌ NO PARAMETRIZADO    │   │
│  │     │  ├─ nav2_params.yaml      ✅ OK                  │   │
│  │     │  └─ moveit_*.yaml         ✅ OK                  │   │
│  │     ├─ scripts/                                         │   │
│  │     │  ├─ joy_teleop.py         ✅ LISTO               │   │
│  │     │  ├─ joint_state_aggregator.py ✅ LISTO          │   │
│  │     │  └─ robot_description_publisher.py ✅ LISTO    │   │
│  │     ├─ worlds/                                          │   │
│  │     │  └─ minimal.world.sdf     (ambiente simple)      │   │
│  │     ├─ models/                                          │   │
│  │     │  └─ warehouse.sdf, furniture, etc.               │   │
│  │     └─ rviz/mm_display.rviz                             │   │
│  │                                                          │   │
│  │  install/                (build artifacts)              │   │
│  │  build/                  (compilación CMake)            │   │
│  │                                                          │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Gazebo Harmonic (Physics Engine)                       │   │
│  │  ├─ /world/minimal                                     │   │
│  │  │  ├─ /mm_base (Rigid body + physics)                │   │
│  │  │  ├─ /mm_arm (Articulated chain)                     │   │
│  │  │  └─ /ground_plane                                   │   │
│  │  └─ Sensores:                                          │   │
│  │     ├─ /mm_base/scan (LaserScan)                       │   │
│  │     ├─ /mm_base/camera (Image RGB)                     │   │
│  │     ├─ /mm_base/nav_camera (Image RGB)                 │   │
│  │     ├─ /mm_base/imu (Imu)                              │   │
│  │     └─ /mm_arm/camera (Image RGB)                      │   │
│  │                                                          │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  ROS 2 Nodos (control + comunicación)                  │   │
│  │                                                          │   │
│  │  ▲ Teleop Layer                                        │   │
│  │  ├─ joy_node (si input:=joystick)                      │   │
│  │  ├─ joy_teleop.py (mapeo joystick → comandos)          │   │
│  │  └─ (solicitudes de movimiento)                        │   │
│  │                                                          │   │
│  │  ▼ Control Layer                                        │   │
│  │  ├─ controller_manager (ROS 2 control)                 │   │
│  │  │  ├─ mecanum_drive_controller (base)                 │   │
│  │  │  ├─ arm_trajectory_controller (brazo)               │   │
│  │  │  ├─ joint_state_broadcaster                         │   │
│  │  │  └─ gripper_action_controller (MoveIt)              │   │
│  │  └─ (envía JointState/TwistStamped)                    │   │
│  │                                                          │   │
│  │  ▼ Simulation Layer                                     │   │
│  │  ├─ gz (Gazebo Harmonic)                               │   │
│  │  ├─ ros2_gz_bridge (topics ROS↔GZ)                     │   │
│  │  └─ (sensores + dinámica)                              │   │
│  │                                                          │   │
│  │  ▲ Navigation + Planning (futuro)                      │   │
│  │  ├─ move_group (MoveIt 2)                              │   │
│  │  ├─ nav2_lifecycle_manager (Nav2)                      │   │
│  │  └─ planner_server                                     │   │
│  │                                                          │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Micro-ROS Agent (para hardware ESP32 futuro)          │   │
│  │  ├─ Escucha UDP:8888 para mensajes de ESP32            │   │
│  │  └─ Publica /joy en ROS 2                              │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────┐
        │   Host: navegador en localhost   │
        │   http://localhost:8080 (noVNC)  │
        │   - RViz 3D viewer               │
        │   - Gazebo GUI (opcional)        │
        │   - Terminal para ros2 commands  │
        └──────────────────────────────────┘
```

---

## 📊 FLUJO DE DATOS EN TIEMPO REAL

```
┌──────────────────────────────────────────────────────────────────────┐
│                   TELEOP → MOVIMIENTO DEL ROBOT                       │
└──────────────────────────────────────────────────────────────────────┘

INPUT (Usuario):
  Joystick / Teclado GUI
           │
           ▼
  /joy (sensor_msgs/Joy)
  QoS: SensorDataQoS (best_effort, depth=1)
           │
           ▼
  joy_teleop.py
  - Lee /joy
  - Mapea según joy_teleop.yaml
  - Aplica deadband, saturación
  - Watchdog (timeout 0.5s)
           │
           ├─→ /mm_base/cmd_vel (Twist)
           │   └─→ mecanum_drive_controller
           │       └─→ Publica JointVelocity a ruedas
           │
           └─→ /mm_arm/arm_target (JointTrajectory)
               └─→ arm_trajectory_controller
                   └─→ Publica JointState de articulos

OUTPUT (Simulación):
  Gazebo recibe órdenes de control
           ├─→ Simula física
           └─→ Actualiza posiciones/orientaciones
                    │
                    ▼
           Publica /tf (transformaciones)
           Publica /joint_states
                    │
                    ▼
           ROS 2 nodos consumen datos:
           ├─ RViz (visualización 3D)
           ├─ MoveIt 2 (planificación)
           ├─ Nav2 (navegación)
           └─ Joy teleop (feedback)
```

---

## 🔄 CICLO DE ARRANQUE (Startup Sequence)

```
1. HOST: docker compose up -d
   └─→ Crea contenedor, inicia supervisord

2. SUPERVISORD arranca servicios:
   ├─ Xvfb (virtual framebuffer)
   ├─ x11vnc (servidor VNC)
   ├─ noVNC (websocket HTML5)
   ├─ XFCE4 (escritorio)
   └─ Bash (espera comandos)

3. USER ejecuta en host:
   docker compose exec -T ros2-vnc bash -lc '
   source /opt/ros/jazzy/setup.bash
   source /home/ros/ros2_ws/install/setup.bash
   ros2 launch mm_bringup modes.launch.py
   '

4. modes.launch.py:
   ├─ DeclareLaunchArgument (40+ parámetros)
   ├─ launch_sim:=true?
   │  └─→ Include sim.launch.py
   │      ├─ Genera URDF/SDF desde Xacro
   │      ├─ Inicia Gazebo
   │      ├─ Espera servicio /world/<name>/control
   │      ├─ Spawns controladores (base + brazo)
   │      └─ Publica robot_description
   │
   ├─ launch_rviz:=true?
   │  └─→ Include display.launch.py
   │      ├─ Genera URDF temporal
   │      ├─ Inicia RViz
   │      └─ Carga mm_display.rviz
   │
   ├─ launch_moveit:=true?
   │  └─→ Include moveit.launch.py
   │      ├─ Carga SRDF ❌ PROBLEMA: HARDCODEADO
   │      ├─ Inicia move_group
   │      ├─ Inicia Servo node
   │      └─ Configura planning scene
   │
   ├─ launch_nav2:=true?
   │  └─→ Include nav2.launch.py
   │      └─ Carga nav2_params + mapa
   │
   └─ launch_teleop:=true?
      └─→ Inicia joy_teleop.py
          ├─ Suscribe /joy
          ├─ Publica /mm_base/cmd_vel
          └─ Publica /mm_arm/arm_target

5. Estado final (normal):
   ✅ Gazebo corriendo
   ✅ RViz mostrando robot
   ✅ Controllers activos
   ✅ Teleop escuchando joystick
   ✅ Sensores simulados publicando
```

---

## 📁 ESTRUCTURA DE ARCHIVOS CRÍTICOS

```
ros2-sim-vnc/
├─ Dockerfile                      (imagen Docker multi-arch)
├─ docker-compose.yml              (servicios: ros2-vnc + micro-ros-agent)
├─ supervisord.conf                (entrypoint servicios)
├─ docker/xorg-dummy.conf          (configuración X11 sin GPU)
│
├─ ros2_ws/src/
│  ├─ mm_base_description/
│  │  └─ urdf/mm_base.urdf.xacro   ✅ PARAMETRIZADO (scale, prefix)
│  │
│  ├─ mm_arm_description/
│  │  └─ urdf/mm_arm.urdf.xacro    ✅ PARAMETRIZADO
│  │
│  └─ mm_bringup/
│     ├─ launch/
│     │  └─ modes.launch.py        ✅ ENTRYPOINT (pero sin validación de rutas)
│     │
│     ├─ config/
│     │  ├─ bridge_params.yaml     ✅ OK
│     │  ├─ base_controllers.yaml  ❌ HARDCODEADO (no escala)
│     │  ├─ arm_controllers.yaml   ✅ OK
│     │  ├─ mm_arm.srdf            ❌ NO SINCRONIZADO con Xacro
│     │  └─ joy_teleop.yaml        ❌ HARDCODEADO (joint names)
│     │
│     ├─ scripts/
│     │  ├─ joy_teleop.py          ✅ LISTO
│     │  ├─ joint_state_aggregator.py
│     │  └─ robot_description_publisher.py
│     │
│     └─ worlds/
│        └─ minimal.world.sdf
│
├─ docs/
│  ├─ arquitectura_moveit_nav2.md
│  ├─ estructura_pseudocodigo.md
│  ├─ patrones_top_tier_ros2.md
│  └─ diagramas/ (drawio files)
│
├─ extras/
│  └─ esp32_funduino_joy/
│     └─ esp32_funduino_joy.ino    (firmware micro-ROS, futuro)
│
├─ .github/workflows/
│  └─ ci.yaml                      ✅ GitHub Actions (build + lint)
│
├─ README.md                       ✅ Documentado
├─ CHANGELOG.md                    ✅ Historial
├─ CONTRIBUTING.md                ✅ Guía colaboradores
├─ AUDIT_REPORT.md                ✅ Análisis completo
├─ PROBLEMAS_ACTUALES.md          ✅ NUEVO (este análisis)
└─ info.txt                        ✅ Comandos rápidos
```

---

## 🟢 LO QUE ESTÁ BIEN

| Componente | Estado | Razón |
|-----------|--------|--------|
| Arquitectura general | ✅ | Modular, escalable, buenas prácticas |
| Scripts Python | ✅ | Ejecutables, type hints, manejo de errores |
| Docker/CI | ✅ | Multi-arquitectura, GitHub Actions presente |
| Documentación | ✅ | README, AUDIT_REPORT, CONTRIBUTING completos |
| Gazebo + RViz | ✅ | Simulación funcional, sensores integrados |
| Teleop básico | ✅ | joystick → base + brazo + gripper |
| QoS | ✅ | Explícito y apropiado para sensores |

---

## 🔴 LO QUE ESTÁ MAL

| Componente | Estado | Por Qué |
|-----------|--------|---------|
| SRDF | ❌ | Hardcodeado, no sincronizado con Xacro |
| Controllers | ❌ | Parámetros hardcodeados, no escalan |
| Joint names | ❌ | Replicados en YAML, no dinámicos |
| Validación rutas | ❌ | Sin comprobación de archivos |
| Tests | ❌ | Sin launch_testing ni integración |
| Versiones | ❌ | Sin pinning exacto en package.xml |

---

## 🎯 PRÓXIMAS ACCIONES (POR URGENCIA)

```
NOW (Hoy)
├─ Leer PROBLEMAS_ACTUALES.md (este archivo)
├─ Entender mapa de arquitectura
└─ Revisar P1, P2, P3

THIS WEEK
├─ Validar rutas en launch files (P3)
├─ Generar SRDF dinámicamente (P1)
└─ Sincronizar controllers (P2)

NEXT WEEK
├─ Fijar versiones en package.xml (M1)
├─ Agregar launch_testing (M3)
└─ Documentar QoS inline (M5)
```

