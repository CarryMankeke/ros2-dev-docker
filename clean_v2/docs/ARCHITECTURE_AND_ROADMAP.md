# ARCHITECTURE AND ROADMAP

## A) Resumen ejecutivo (core vs opt-in y multi-robot)
- El proyecto clean_v2 se organiza con un core estable y modulos opt-in desacoplados.
- Core estable: Gazebo + /clock + TF + ros2_control + robot_state_publisher + sensores base.
- Opt-in: MoveIt, Nav2, teleop/ESP32, RViz MotionPlanning.
- Si un modulo opt-in falta, el core no debe caer ni bloquear el arranque.
- El core usa un solo robot_description y un solo arbol TF por robot.
- Multi-robot obligatorio: namespaces /mm1 y /mm2 para topics y nodos.
- Frames unicos por prefijo: mm1_* y mm2_*; /tf es global sin colisiones.
- URDF/Xacro modular: base y brazo como macros, mm_robot como ensamblador.
- Los controladores se generan desde templates por launch, no a mano.
- MoveIt y Nav2 se lanzan aparte (opt-in) y no deben levantar Gazebo.
- La higiene del repo exige ignorar build/install/log y evitar artefactos.
- Este documento combina arquitectura, auditoria con evidencia y roadmap M0..M6.

## B) Estado actual del repo (paquetes existentes)
- mm_base_description
  - Provee: URDF/Xacro de base, ruedas y sensores base.
  - No debe proveer: launch, controladores, MoveIt, Nav2.
- mm_arm_description
  - Provee: URDF/Xacro del brazo y accesorios.
  - No debe proveer: ensamblaje del robot ni launch.
- mm_robot_description
  - Provee: ensamblaje del robot completo (base + brazo + sensores).
  - No debe proveer: launch ni configuracion MoveIt/Nav2.
- mm_bringup
  - Provee: launch core, templates de controladores, worlds, configs RViz, scripts.
  - No debe proveer: URDF del robot ni config MoveIt.
- mm_moveit_config
  - Provee: configuracion y launch de MoveIt (opt-in).
  - No debe proveer: Gazebo, ros2_control ni controladores del core.

## C) Entrypoints reales HOY (lista exacta con evidencia)

### clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py
- Tipo: core parcial (descripcion + robot_state_publisher, sin Gazebo).
- URDF: mm_base.urdf.xacro (clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py:45).
- YAML: base_controllers.yaml.in (clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py:22).
- World: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "world" clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py
- RViz: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "rviz" clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py

### clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py
- Tipo: core minimo con Gazebo.
- URDF: mm_base.urdf.xacro (clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py:60).
- YAML: base_controllers.yaml.in (clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py:23).
- World: minimal.world.sdf (clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py:68).
- RViz: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "rviz" clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py

### clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py
- Tipo: core completo (Gazebo + robot completo + RViz).
- URDF: mm_robot.urdf.xacro (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:214).
- YAML: mm_controllers.yaml.in (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:33).
- World: minimal.world.sdf (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:223).
- RViz template bug confirmado:
  - template_name = 'mm_verify.rviz.in' if rviz_mode == 'verify' else 'mm_verify.rviz.in'
  - Evidencia: clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:53

### clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py
- Tipo: core multi-robot (mm1 + mm2).
- URDF: mm_robot.urdf.xacro (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py:197).
- YAML: mm_controllers.yaml.in (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py:27).
- World: minimal.world.sdf (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py:211).
- RViz: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "rviz" clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py

### clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py
- Tipo: opt-in (Nav2).
- YAML: nav2_params.yaml.in (clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py:25).
- URDF: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "urdf" clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py
- RViz: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "rviz" clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py

### clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py
- Tipo: opt-in (MoveIt).
- URDF: mm_robot.urdf.xacro (clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py:46).
- SRDF: mm_robot.srdf.xacro (clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py:51).
- YAML renderizados: moveit_controllers.yaml.in y joint_limits.yaml.in (clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py:28).
- YAML cargados: kinematics, ompl_planning, planning_scene_monitor, trajectory_execution
  - clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py:35-38
- RViz: NO REFERENCIADO (por nombre). Confirmar:
  - rg -n "rviz" clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py

### mm_*_description/launch
- No hay launch en mm_base_description, mm_arm_description, mm_robot_description.
- NECESITA CONFIRMACION:
  - rg --files clean_v2/ros2_ws/src/mm_*_description/launch

## D) Contrato TF y topics (mm1/mm2)

### TF (core, requerido)
- mm1_odom -> mm1_base_footprint (dinamico, controlador de base).
- mm1_base_footprint -> mm1_base_link (estatico, URDF).
  - Evidencia base_footprint: clean_v2/ros2_ws/src/mm_base_description/urdf/mm_base_macro.xacro:32,57
- mm1_base_link -> mm1_lidar_link (estatico, URDF).
  - Evidencia lidar_link: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:136
- mm1_base_link -> mm1_cam_front_link / mm1_cam_left_link / mm1_cam_right_link / mm1_cam_rear_link
  - Evidencia cam links: clean_v2/ros2_ws/src/mm_base_description/urdf/mm_base_macro.xacro:155,177,199,221
- mm1_tool0 -> mm1_ee_cam_mount_link -> mm1_ee_cam_link
  - Evidencia ee_cam links: clean_v2/ros2_ws/src/mm_arm_description/urdf/mm_arm_macro.xacro:238,260
- mm1_base_link -> mm1_imu_link (estatico, URDF).
  - Evidencia imu_link: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:193-220

### IMU EE (propuesto)
- mm1_tool0 -> mm1_ee_imu_link (propuesto, no existe hoy).
- NECESITA CONFIRMACION:
  - rg -n "ee_imu|imu_ee" clean_v2/ros2_ws/src

### Topics (namespaced, recomendados)
- Lidar: /mm1/scan y /mm2/scan
  - Evidencia: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:74,187
- IMU base: /mm1/imu y /mm2/imu
  - Evidencia: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:227
- Camaras base:
  - /mm1/camera/front/image_raw
  - /mm1/camera/left/image_raw
  - /mm1/camera/right/image_raw
  - /mm1/camera/rear/image_raw
  - Evidencia: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:75-78
- Camara EE: /mm1/camera/ee/image_raw
  - Evidencia: clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro:79
- IMU EE (propuesto): /mm1/imu/ee (no existe hoy).

## E) Que mas considerar (Jazzy + Harmonic)
- /clock y use_sim_time: un solo publisher de /clock en core; todos los nodos usan use_sim_time.
- QoS recomendado:
  - IMU/lidar/camaras: SensorDataQoS (best_effort, depth 5).
  - /tf: default QoS de tf2, /tf_static: transient_local.
- Namespacing estricto:
  - Topics: /mm1/... y /mm2/...
  - Frames: mm1_* y mm2_* (sin duplicados).
- Fallback seguridad:
  - cmd_vel watchdog publica cero si no hay teleop o Nav2.
  - teleop y Nav2 opt-in: si falta, no tumbar core.
- Separacion sim vs real:
  - plugins Gazebo y bridges solo en sim.
  - External drivers se conectan via opt-in sin tocar el core.
- Performance:
  - throttle camaras si CPU alta.
  - ajustar frecuencia IMU y lidar en sim si hay overload.
- Health checks:
  - TF tree, controllers activos, sensores publicando, /clock monotono.
  - usar scripts smoke_* bajo mm_bringup/scripts.

## External references (course notes)
Estos archivos son material de referencia externo. No son contrato del stack.
- clean_v2/docs/reference/README.md
- clean_v2/docs/reference/course/MoveIt_2.md
- clean_v2/docs/reference/course/MoveIt_2_MTC.md
- clean_v2/docs/reference/course/Moveit_2_Planning_Scene.md
- clean_v2/docs/reference/course/Moveit_2_Visual_Tools.md
- clean_v2/docs/reference/course/Moveit_2_C++_API.md
- clean_v2/docs/reference/course/Nav2.md

## F) Auditoria verificable (tabla)

| Archivo | Tipo | Evidencia de uso | Impacto | Accion |
|---|---|---|---|---|
| clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py | launch | clean_v2/ros2_ws/src/mm_bringup/launch/base_min.launch.py:45 ("mm_base.urdf.xacro"), :22 ("base_controllers.yaml.in") | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py | launch | clean_v2/ros2_ws/src/mm_bringup/launch/sim_min.launch.py:60 ("mm_base.urdf.xacro"), :23 ("base_controllers.yaml.in"), :68 ("minimal.world.sdf") | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py | launch | clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:214 ("mm_robot.urdf.xacro"), :33 ("mm_controllers.yaml.in"), :223 ("minimal.world.sdf"), :53 (rviz template bug) | alto | corregir bug rviz_mode |
| clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py | launch | clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm_dual.launch.py:197 ("mm_robot.urdf.xacro"), :27 ("mm_controllers.yaml.in"), :211 ("minimal.world.sdf") | alto | mantener |
| clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py | launch | clean_v2/ros2_ws/src/mm_bringup/launch/nav2_min.launch.py:25 ("nav2_params.yaml.in") | medio | mantener (opt-in) |
| clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py | launch | clean_v2/ros2_ws/src/mm_moveit_config/launch/moveit.launch.py:46 ("mm_robot.urdf.xacro"), :51 ("mm_robot.srdf.xacro"), :28 (templates), :35-38 (yaml) | alto | mantener (opt-in) |
| clean_v2/ros2_ws/src/mm_bringup/config/base_controllers.yaml.in | yaml | Referenciado en base_min.launch.py:22 y sim_min.launch.py:23 | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/config/mm_controllers.yaml.in | yaml | Referenciado en sim_mm.launch.py:33 y sim_mm_dual.launch.py:27; incluye base+arm+gripper (clean_v2/ros2_ws/src/mm_bringup/config/mm_controllers.yaml.in:9,12,15) | alto | mantener |
| clean_v2/ros2_ws/src/mm_bringup/config/nav2_params.yaml.in | yaml | Referenciado en nav2_min.launch.py:25 | medio | mantener (opt-in) |
| clean_v2/ros2_ws/src/mm_bringup/config/arm_controllers.yaml.in | yaml | NO REFERENCIADO (por nombre). Confirmar: rg -n "arm_controllers.yaml.in" clean_v2/ros2_ws/src | medio | eliminar o mover a docs/legacy |
| clean_v2/ros2_ws/src/mm_bringup/rviz/mm_verify.rviz.in | rviz | Usado en sim_mm.launch.py por template (clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:53) | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/rviz/mm_display.rviz.in | rviz | NO USADO por bug en seleccion de template. Evidencia del bug: clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:53 | medio | integrar o eliminar |
| clean_v2/ros2_ws/src/mm_bringup/worlds/minimal.world.sdf | world | Referenciado en sim_min.launch.py:68, sim_mm.launch.py:223, sim_mm_dual.launch.py:211 | medio | mantener |
| clean_v2/ros2_ws/src/mm_robot_description/urdf/mm_robot.urdf.xacro | xacro | Referenciado en sim_mm.launch.py:214, sim_mm_dual.launch.py:197, moveit.launch.py:46 | alto | mantener |
| clean_v2/ros2_ws/src/mm_base_description/urdf/mm_base.urdf.xacro | xacro | Referenciado en base_min.launch.py:45, sim_min.launch.py:60 | medio | mantener |
| clean_v2/ros2_ws/src/mm_base_description/urdf/mm_base_macro.xacro | xacro | Referenciado por include (clean_v2/ros2_ws/src/mm_base_description/urdf/mm_base.urdf.xacro:4) | medio | mantener |
| clean_v2/ros2_ws/src/mm_arm_description/urdf/mm_arm.urdf.xacro | xacro | NO REFERENCIADO (por nombre). Confirmar: rg -n "mm_arm.urdf.xacro" clean_v2/ros2_ws/src | bajo | revisar si es legacy |
| clean_v2/ros2_ws/src/mm_arm_description/urdf/mm_arm_macro.xacro | xacro | Referenciado por include (clean_v2/ros2_ws/src/mm_arm_description/urdf/mm_arm.urdf.xacro:4) | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/mm_robot.srdf.xacro | srdf | Referenciado en moveit.launch.py:51 | alto | mantener |
| clean_v2/ros2_ws/src/mm_arm_description/srdf/mm_arm.srdf.xacro | srdf | NO REFERENCIADO (por nombre). Confirmar: rg -n "mm_arm.srdf.xacro" clean_v2/ros2_ws/src | medio | revisar si es legacy |
| clean_v2/ros2_ws/src/mm_moveit_config/config/ompl_planning.yaml | yaml | Cargado en moveit.launch.py:36 | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/kinematics.yaml | yaml | Cargado en moveit.launch.py:35 | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/moveit_controllers.yaml.in | yaml | Renderizado en moveit.launch.py:28 | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/joint_limits.yaml.in | yaml | Renderizado en moveit.launch.py:28 | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/planning_scene_monitor_parameters.yaml | yaml | Cargado en moveit.launch.py:37 | medio | mantener |
| clean_v2/ros2_ws/src/mm_moveit_config/config/trajectory_execution.yaml | yaml | Cargado en moveit.launch.py:38 | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/scripts/run_smoke_tests.sh | script | Instalado por CMakeLists (clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17) | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/scripts/smoke_tf.py | script | Instalado por CMakeLists (clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17) | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/scripts/smoke_controllers.py | script | Instalado por CMakeLists (clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17) | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/scripts/smoke_cameras.py | script | Instalado por CMakeLists (clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17) | medio | mantener |
| clean_v2/ros2_ws/src/mm_bringup/scripts/smoke_sim_time.py | script | Instalado por CMakeLists (clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17) | medio | mantener |
| clean_v2/docs/audit/review_summary_20260112.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "review_summary_20260112" clean_v2 | bajo | revisar y reducir |
| clean_v2/docs/legacy/problemas_actuales.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "problemas_actuales" clean_v2 | bajo | archivar |
| clean_v2/docs/legacy/analisis_completado.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "analisis_completado" clean_v2 | bajo | archivar |
| clean_v2/docs/legacy/implementacion_completa.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "implementacion_completa" clean_v2 | bajo | archivar |
| clean_v2/docs/legacy/plan_accion_inmediato.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "plan_accion_inmediato" clean_v2 | bajo | archivar |
| clean_v2/docs/reference/README.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "docs/reference/README.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/MoveIt_2.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "MoveIt_2.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/MoveIt_2_MTC.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "MoveIt_2_MTC.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/Moveit_2_Planning_Scene.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "Moveit_2_Planning_Scene.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/Moveit_2_Visual_Tools.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "Moveit_2_Visual_Tools.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/Moveit_2_C++_API.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "Moveit_2_C++_API.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/docs/reference/course/Nav2.md | doc | NO REFERENCIADO (por nombre). Confirmar: rg -n "Nav2.md" clean_v2 | bajo | mantener (referencia externa) |
| clean_v2/ros2_ws/build/ | build-artifact | Existe (ej: clean_v2/ros2_ws/build/mm_bringup/CMakeCache.txt). Confirmar: rg --files clean_v2/ros2_ws/build | alto | gitignore |
| clean_v2/ros2_ws/install/ | build-artifact | Existe (ej: clean_v2/ros2_ws/install/setup.bash). Confirmar: rg --files clean_v2/ros2_ws/install | alto | gitignore |
| clean_v2/ros2_ws/log/ | build-artifact | Existe (ej: clean_v2/ros2_ws/log/COLCON_IGNORE). Confirmar: rg --files clean_v2/ros2_ws/log | alto | gitignore |

## G) Fixes propuestos (basados en evidencia)

1) Corregir rviz_mode en sim_mm.launch.py
- Evidencia bug: clean_v2/ros2_ws/src/mm_bringup/launch/sim_mm.launch.py:53
- Snippet propuesto:

```python
if rviz_mode == 'verify':
    template_name = 'mm_verify.rviz.in'
elif rviz_mode == 'display':
    template_name = 'mm_display.rviz.in'
else:
    template_name = 'mm_verify.rviz.in'
```

arm_controllers.yaml.in
- mm_controllers.yaml.in ya incluye base, brazo y gripper (clean_v2/ros2_ws/src/mm_bringup/config/mm_controllers.yaml.in:9,12,15).
- arm_controllers.yaml.in no se usa por ningun launch (NO REFERENCIADO por nombre).
- Recomendacion: mover a clean_v2/docs/legacy/ o eliminar para evitar duplicidad y confusion.

Scripts en mm_bringup/scripts
- Evidencia de instalacion: clean_v2/ros2_ws/src/mm_bringup/CMakeLists.txt:10-17 (install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})).
- Ejecucion: ros2 run mm_bringup smoke_tf.py (ejemplo).
- No asumir referencias en docs si no hay evidencia directa.

## H) Higiene del repo
- Politica: versionar solo clean_v2/ros2_ws/src y clean_v2/docs.
- Ignorar build/install/log.
- Snippet recomendado de .gitignore:

```
clean_v2/ros2_ws/build/
clean_v2/ros2_ws/install/
clean_v2/ros2_ws/log/
.DS_Store
```

## I) Roadmap M0..M6 (DoD)

### M0: estructura y core estable
- [ ] sim_min.launch.py levanta Gazebo + /clock + TF base.
- [ ] sim_mm.launch.py levanta robot completo con ros2_control.
- [ ] /clock tiene 1 publisher y use_sim_time activo.

### M1: sensores base + TF completo
- [ ] lidar publica /mm1/scan.
- [ ] camaras base publican /mm1/camera/*/image_raw.
- [ ] TF incluye cam_*_link y lidar_link.

### M2: control base + seguridad
- [ ] omni_wheel_controller activo en /mm1/controller_manager.
- [ ] cmd_vel watchdog publica cero en ausencia de input.
- [ ] core sigue estable sin teleop.

### M3: brazo + gripper control
- [ ] arm_trajectory_controller activo y responde FollowJointTrajectory.
- [ ] gripper_trajectory_controller activo y responde.
- [ ] TF del brazo estable (sin flicker).

### M4: MoveIt opt-in
- [ ] moveit.launch.py arranca sin Gazebo.
- [ ] MoveIt usa /mm1 y ejecuta FollowJointTrajectory.
- [ ] planifica y ejecuta en RViz con /clock.

### M5: Nav2 opt-in
- [ ] nav2_min.launch.py arranca con params renderizados.
- [ ] core sigue estable si Nav2 no esta instalado.
- [ ] cmd_vel arbitration documentado (teleop vs nav2).

### M6: dual-robot (mm1/mm2)
- [ ] sim_mm_dual.launch.py arranca mm1 y mm2 sin colisiones.
- [ ] /tf y /tf_static con frames unicos (mm1_*, mm2_*).
- [ ] sensores y controladores namespaced correctamente.
