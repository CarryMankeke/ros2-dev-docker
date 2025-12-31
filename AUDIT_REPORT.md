# Reporte Exhaustivo de Auditoría del Proyecto ros2-sim-vnc
**Fecha:** 27 de diciembre de 2025  
**Alcance:** Revisión completa de arquitectura, código, configuración y entorno Docker

---

## I. RESUMEN EJECUTIVO

El proyecto **ros2-sim-vnc** es un stack ROS 2 Jazzy + Gazebo Harmonic bien estructurado para simular un robot móvil omnidireccional (mm_base) con brazo 6DOF (mm_arm) y teleoperación por joystick. La arquitectura es modular, documentada y operativa, pero presenta varios problemas de **permisos de archivo, consistencia de dependencias y cobertura de testing** que deben resolverse antes de producción.

### Puntuación General
- **Arquiectura:** 8/10 (modular, buenas prácticas, pero falta versionado de paquetes)
- **Código:** 7/10 (limpio, con pendientes en tests y parametrización)
- **Documentación:** 8/10 (README y docs completos, faltan versiones exactas)
- **DevOps/CI:** 6/10 (CI básico presente, faltan linters y launch_testing)
- **Infraestructura Docker:** 9/10 (bien configurado, noVNC funcional, multi-arquitectura)

---

## II. HALLAZGOS POR CATEGORÍA

### A. SCRIPTS DE PYTHON

#### ✅ ESTADO ACTUAL
- **`joy_teleop.py`** (15.8 KB): Ejecutable (`-rwxr-xr-x`), bien escrito, completo.
  - Type hints robustos, manejo de parámetros flexible, deadman switches, modos hybrid/base/arm, soporte para gripper y presets home.
  - Implementa watchdog (`joy_timeout`), QoS sensor data, escalas configurables.
  - ✅ LISTO PARA PRODUCCIÓN.

- **`robot_description_publisher.py`** (2.1 KB): Ejecutable (`-rwxr-xr-x`), compacto.
  - Publica URDF desde archivo con retry hasta que exista, QoS TRANSIENT_LOCAL+RELIABLE.
  - ✅ LISTO PARA PRODUCCIÓN.

- **`joint_state_aggregator.py`** (3.0 KB): Ejecutable (`-rwxr-xr-x`).
  - Agrega JointState de base y brazo, implementación correcta.
  - ✅ LISTO PARA PRODUCCIÓN.

#### 🔍 ANÁLISIS DETALLADO
| Script | Shebang | Permisos | Type Hints | Manejo Excepciones | QoS | Estado |
|--------|---------|----------|------------|-------------------|-----|--------|
| joy_teleop.py | ✅ | ✅ | ✅ Robustos | ✅ | ✅ SensorData | ✅ |
| robot_description_publisher.py | ✅ | ✅ | ✅ Parcial | ✅ | ✅ TRANSIENT_LOCAL | ✅ |
| joint_state_aggregator.py | ✅ | ✅ | ✅ | ✅ | ✅ RELIABLE | ✅ |

---

### B. LAUNCHFILES

#### ✅ `modes.launch.py` (269 líneas)
**Punto de entrada principal**: Unifica sim, display, nav2, moveit, teleop bajo un solo comando.
- **Strengths:**
  - Declaración explícita de todos los argumentos (40+).
  - Condicionales bien formados (`IfCondition`).
  - Mapeo configurable de joystick (toy_params).
  - Organización lógica: sim → display → nav2 → moveit → teleop.
- **Debilidades:**
  - Algunas expresiones Python son largas y difíciles de leer (concatenación de strings en `PythonExpression`).
  - No documenta cuáles argumentos son obligatorios vs. opcionales.

#### ✅ `sim.launch.py` (489 líneas)
**Simulación Gazebo + Bringup**: Genera URDF dinámico, lanza Gazebo, spawners de controladores.
- **Strengths:**
  - Xacro generation con parámetros (scale, prefix).
  - SDF assembly dinámico (base + brazo + joint fijo).
  - Encadenamiento de spawners via `RegisterEventHandler` + `OnProcessExit` (correcto).
  - Bridge params desde YAML.
  - Soporte para timezones y GZ_SIM_RESOURCE_PATH.
- **Debilidades:**
  - El arranque de Gazebo ahora espera el servicio `/world/<world_name>/control` antes de spawnear controladores.
  - Si se cambia el nombre del mundo, hay que ajustar `world_name` en el launch.

#### ✅ `moveit.launch.py` (113 líneas)
**MoveIt 2 + Servo + RViz**: Move group, servo node, state publisher.
- **Strengths:**
  - Carga SRDF via `Command(['cat', srdf_path])` (funcional pero no reutiliza xacro).
  - Remap de `/joint_states` explícito.
  - Servo node con twist commands.
- **Debilidades:**
  - **SRDF no se genera desde Xacro**: está hardcodeado a `mm_arm.srdf` (sin parámetros dinámicos). Si cambias prefix, SRDF debe actualizarse manualmente.
  - No declara namespace de MoveIt (runs en global namespace; potencial conflicto si hay otros robots).

#### ✅ `display.launch.py` (259 líneas)
**RViz sin Gazebo**: Genera URDF en `/tmp` para RViz.
- **Strengths:**
  - Soporta fake joint states y GUI.
  - TF estático base->arm configurable.
- **Debilidades:**
  - Genera archivos en `/tmp/mm_*.urdf` (temporal, potencial colisión si hay múltiples instancias).
  - Condición compleja para `use_joint_state_gui` (legible pero frágil).

#### ✅ `nav2.launch.py` (57 líneas)
**Inclusión de Nav2**: Wrapper simple alrededor de `nav2_bringup/bringup_launch.py`.
- **Strengths:**
  - Delegación clara a nav2_bringup.
  - Argumentos mapeados correctamente.
- **Debilidades:**
  - Usa BT por defecto de `nav2_bt_navigator`; si se cambia, no hay validación de archivo.
  - Sin validación de mapa YAML existente.

---

### C. CONFIGURACIÓN (YAML)

#### ✅ `bridge_params.yaml` (48 líneas)
**Mapeos ROS↔Gazebo**:
- Topics mapeados:
  - **Base:** `/mm_base/scan` (LaserScan), `/mm_base/camera` (Image), `/mm_base/nav_camera` (Image), `/mm_base/camera/camera_info` (CameraInfo), `/mm_base/imu` (Imu).
  - **Arm:** `/mm_arm/camera` (Image), `/mm_arm/camera/camera_info` (CameraInfo), `/mm_arm/imu` (Imu).
  - **Clock:** `/clock` (ROS↔Gazebo).
- **✅ CORRECTO** para Gazebo Harmonic (tipos `gz.msgs.*`).
- **⚠️ NOTA:** Si usas OpenGL con OpenGL+SDF muy reciente, los tipos `gz.msgs.Image` pueden cambiar; validar con `ros2 interface list`.

#### ✅ `joy_teleop.yaml` (59 líneas)
**Mapeo de Joystick**:
- **Modo base:** Ejes `[1]` (Y) → vx, `[0]` (X) → vy, `[-1]` (sin eje) → wz via botones.
- **Modo brazo:** Eje dpad `[2,3]` → Δx, Δy; botones `[-1]` → sin control Z.
- **Deadman:** Botones `[4,5]` (L2, R2).
- **Gripper:** Botones `[0,1]` (A, B).
- **Home:** Botón `[6]` (SW).
- **Mode axis:** `[4]` (RB) → cambio base/arm/hybrid.
- **Arm joint names:** Listadas explícitamente (6 joints).
- ✅ **COHERENTE** con `joy_teleop.py` y URDF/descripción del brazo.

#### ✅ `base_controllers.yaml` (44 líneas)
**Controladores de la base (Mecanum)**:
- `controller_manager`: update_rate 100 Hz, use_sim_time=true.
- `joint_state_broadcaster`: estándar.
- `mecanum_drive_controller`: 
  - Wheel names: `mm_base_front_{left,right}_wheel_joint`, `mm_base_rear_{left,right}_wheel_joint`.
  - Separaciones: `wheel_separation_x: 0.33 m`, `wheel_separation_y: 0.33 m`, `wheel_radius: 0.06 m`.
  - Max velocidades: vx/vy 1.0 m/s, wz 1.5 rad/s.
- **⚠️ CRÍTICO:** Comentario **"Mantener estos valores en sincronía con mm_base.urdf.xacro"** pero SIN mecanismo automático.
  - En `mm_base.urdf.xacro`: `wheel_separation_x = 0.33 * scale`, `wheel_separation_y = 0.33 * scale`, `wheel_radius = 0.06 * scale`.
  - **RIESGO:** Si `scale != 1.0`, los valores no están sincronizados.
  - **ACCIÓN RECOMENDADA:** Generar `base_controllers.yaml` desde template en Jinja2 o usar variable global en xacro.

#### ✅ `arm_controllers.yaml` (38 líneas)
**Controladores del brazo (Trajectory)**:
- `arm_trajectory_controller`: 6 joints, command_interface=position, state=position+velocity.
- Constraints por joint: tolerancia trayectoria 0.05 rad, tolerancia goal 0.02 rad.
- ✅ **COHERENTE** con joints del URDF y `joy_teleop.yaml`.

#### ✅ `moveit_servo.yaml` (18 líneas)
**Servo Node (Control Cartesiano Continuo)**:
- Comando entrada: twist, salida: trayectoria.
- Frame de planificación: `mm_arm_root_link`.
- Max velocidades: lineal 0.4 m/s, angular 1.0 rad/s.
- Timeout de comando: 0.25 s.
- ✅ **FUNCIONAL**, pero sin smoothing explícito declarado (usa plugin por defecto).

#### ⚠️ `moveit_controllers.yaml` (17 líneas)
**MoveIt Controller Manager**:
- Define solo `arm_trajectory_controller`, sin gripper.
- **NOTA:** No hay entrada para `gripper_controller` (si deseas manipulación, debes extenderlo).

#### ⚠️ `moveit_planning.yaml` y `moveit_kinematics.yaml`
- **NO REVISADOS** en esta auditoría (asumen defaults de MoveIt 2 Jazzy).

---

### D. URDF/XACRO

#### ✅ `mm_base.urdf.xacro` (315 líneas)
**Base Móvil Omnidireccional (Mecanum)**:
- **Estructura:**
  - Link root: `base_footprint` → `base_link` (offset por altura).
  - Ruedas: 4 joints continuos (front_left, front_right, rear_left, rear_right).
  - Sensores: lidar, cámara frontal, nav_cámara, IMU.
  - ros2_control: sistema tipo `gz_ros2_control/GazeboSimSystem`.
- **Propiedades Escalables:**
  - `base_length/width/height` escalables por `scale`.
  - `wheel_radius`, separaciones (x, y) escalables.
  - Parámetros de sensor (FOV, actualizaciones) fijos.
- **Inertias:** Valores razonables para simulación (0.18 en xx/yy, 0.22 en zz para base; 0.002 para ruedas).
- **Gazebo Plugins:**
  - GPU LiDAR: 640 samples, FOV 160°, 10 Hz.
  - Cámaras: 640×480@30 Hz.
  - IMU: 100 Hz.
  - `gz_ros2_control-system.so` + plugin ROS2Control.
- **✅ CALIDAD:** Bien estructurado, prefijos correctos, macros usadas apropiadamente.
- **⚠️ NOTA:** No hay colisiones entre ruedas y base (cuerpo simple); en robot real necesitaría ajuste.

#### ✅ `mm_arm.urdf.xacro` (377 líneas)
**Brazo 6DOF + Gripper + Sensores**:
- **Estructura:**
  - Root: `root_link` → `base_link` (offset vertical).
  - Cadena cinemática: 6 revolute joints (shoulder_pan, shoulder_lift, elbow, wrist_1/2/3).
  - Efector: `tool0` + `gripper_link` + cámara EE + IMU EE.
  - ros2_control: 6 joints con interfaces posición (command) y posición+velocidad (state).
- **Rangos Articulares:**
  - Shoulder_pan: [-π, π] (360°).
  - Shoulder_lift: [-π/2, π/2].
  - Elbow: [-π/2, π/2].
  - Wrist_*: [-π, π] (wrist_1 en eje X, wrist_2 en eje Y, wrist_3 en eje X).
  - Esfuerzos: 30 N⋅m (shoulder), 25 N⋅m (elbow), 20 N⋅m (wrist), 15 N⋅m (wrist_3).
- **Escalado:** Geometría de links escalable, inertias proporcionales.
- **Sensores:**
  - EE Camera: 640×480@30 Hz, FOV 60°, offset cartesiano (0.02, 0, 0.06) m.
  - EE IMU: 100 Hz.
- **✅ CALIDAD:** Bien balanceado, límites coherentes, dimensiones realistas.
- **⚠️ LIMITACIÓN:** Gripper es joint fijo (sin actuadores); modelo simplificado para esta fase.

#### ❌ ISSUES CROSS-CHECKING XACRO ↔ CONFIG
1. **`base_controllers.yaml` NO se regenera** si cambias `mm_base.urdf.xacro`:
   - Ej: Si pones `scale:=2.0`, `wheel_separation_x` se duplica en URDF pero NO en YAML.
   - **RIESGO CRÍTICO:** Discrepancia en cinemática directa → odometría incorrecta.
   - **SOLUCIÓN:** Template Jinja2 o extraer parámetros dinámicamente.

2. **SRDF no parametrizado:** `mm_arm.srdf` es estático; no soporta `scale` o `prefix` dinámico.
   - Si cambias `arm_prefix:=custom_arm_`, MoveIt fallará (SRDF todavía cita `mm_arm_*`).

---

### E. EMPAQUETADO Y MANIFESTS

#### ✅ `package.xml` (todos los 3 paquetes)
| Paquete | Maintainer | Licencia | BuildTool | Deps Clave |
|---------|-----------|----------|-----------|-----------|
| mm_bringup | Camilo Soto Villegas | BSD-3-Clause | ament_cmake | launch, launch_ros, xacro, robot_state_publisher, rviz2, controller_manager, ros_gz_*, rclpy, msgs |
| mm_base_description | Camilo Soto Villegas | BSD-3-Clause | ament_cmake | xacro |
| mm_arm_description | Camilo Soto Villegas | BSD-3-Clause | ament_cmake | xacro |

**Observaciones:**
- ✅ **Metadata actualizado:** Email y licencia correctos.
- ⚠️ **Sin versión exacta de paquetes:** Dependencies sin `~=` o `>=` versions.
  - Ejemplo: `<exec_depend>ros_gz_sim</exec_depend>` (cualquier versión).
  - **RECOMENDACIÓN:** Usar versiones exactas o rangos en producción (ej: `ros2_control (>=2.25,<3.0)`).
- ✅ **`test_depend`:** Incluye `ament_lint_auto` y `ament_lint_common`.

#### ✅ `CMakeLists.txt`
| Paquete | Contenido | Lint Config |
|---------|-----------|------------|
| mm_bringup | Instala launch/, config/, worlds/, rviz/, models/; scripts/ via `install(PROGRAMS)` | Desactiva cpplint/copyright condicionalmente |
| mm_base_description | Instala urdf/ | Default |
| mm_arm_description | Instala urdf/ | Default |

**Issues:**
- ⚠️ **Scripts sin entry points:** Instalados via `PROGRAMS` (funciona pero menos elegante que entry_points en setup.py).
  - **Mejor práctica Jazzy:** Usar `<exec_depend>python3-rosdebian</exec_depend>` + ament_python si posible, pero ament_cmake también es válido.

---

### F. INFRAESTRUCTURA DOCKER

#### ✅ `Dockerfile` (65 líneas)
**Base:** `ros:jazzy-ros-base-noble` (multi-arch: amd64, arm64).
- **Paquetes instalados:**
  - Entorno gráfico: xfce4, x11vnc, novnc, websockify, supervisor.
  - ROS Jazzy: desktop-full, ros_gz, ros2_control, ros2_controllers, gz_ros2_control, moveit, nav2, teleop_twist_keyboard.
- **Variables de entorno:** DISPLAY, QT_X11_NO_MITSHM, LIBGL_ALWAYS_SOFTWARE (para OpenGL sin GPU).
- ✅ **APROPIADO:** Multi-arch, basado en noble (LTS), bem configurado para VNC.
- ⚠️ **Mejorable:**
  - Sin `HEALTHCHECK`.
  - Sin `USER` (runs como root, potencial riesgo de seguridad en producción).

#### ✅ `docker-compose.yml` (52 líneas)
- **Servicios:**
  - `ros2-vnc`: Container ROS con entorno gráfico, volume mount de `ros2_ws/`, port 8080 (noVNC).
  - `micro-ros-agent`: Contenedor micro-ROS Jazzy, UDP port 8888.
- **Red:** Bridge `rosnet` para comunicación entre contenedores.
- **Volúmenes:** `./ros2_ws:/home/ros/ros2_ws:rw` (bind mount).
- ✅ **CORRECTO:** ROS_DOMAIN_ID=0, RMW_IMPLEMENTATION=rmw_fastrtps_cpp, shm_size=512m (adecuado).
- ⚠️ **Mejoras:**
  - Sin `depends_on` (micro-ros-agent podría iniciar antes de ros2-vnc).
  - Sin healthcheck (servicios siempre `unless-stopped` sin validación).

#### ✅ `supervisord.conf` (44 líneas)
**Procesos supervisionados:**
- Xvfb (X virtual frame buffer).
- Xfce4 (desktop).
- x11vnc (VNC server).
- noVNC (web interface).
- ✅ **FUNCIONAL:** Inicia servicios en orden correcto (priority 5 → 10 → 15 → 20).
- ⚠️ **SIN micro-ROS Agent:** No aparece en supervisord; probablemente se inicia via docker-compose `command`.
  - MEJOR: Agregar sección `[program:micro-ros-agent]` en supervisord si quieres control centralizado.

---

### G. DOCUMENTACIÓN

#### ✅ `README.md` (excelente)
- Flujo rápido de 3 pasos (build, setup, launch).
- Estructura clave con referencias a carpetas.
- Lanzamientos sugeridos con argumentos útiles.
- Comandos copyable.
- **PERO:** Sin indicación de **versión exacta de dependencias** (ROS 2.1.10? Gazebo 9.x?).

#### ✅ `info.txt` (muy útil)
- Guía rápida de comandos docker-compose y modos de lanzamiento.
- Mapeo de joystick documentado (ejes, botones).
- Troubleshooting básico.
- **TODO:** Agregar sección de requirements (HW mínimo, versiones exactas).

#### ✅ `docs/estructura_pseudocodigo.md` (239 líneas)
- Esqueleto de arquitectura en pseudocódigo (NO ejecutable).
- Excelente para entender la intención del sistema.
- Buena documentación de modos, launchfiles, seguridad, watchdog.

#### ✅ `docs/arquitectura_moveit_nav2.md` (194 líneas)
- Detalles sobre MoveIt 2, Nav2, frames, bridges.
- Descripción de controladores y QoS.
- Bueno pero **falta de links a archivos reales** en ros2_ws.

#### ❌ `AGENTS.md` (12 líneas)
- Lineamientos de contribución; MÁS BIEN es un CONFIG que documentación del proyecto.
- Bien escrito pero fuera de lugar.

#### ⚠️ **FALTAN:**
- **REQUIREMENTS.txt** o **versiones exactas** en docs/setup.
- **API docs:** Sin sphinx/docstring generación.

---

### H. TESTING E INTEGRACIÓN CONTINUA

#### ✅ **CI/CD básico presente**
- Existe `.github/workflows/ci.yaml` con `colcon build` y `colcon test`.
- Aún faltan linters y `launch_testing` dedicados.

**Pendiente para producción:**
- Agregar linting (ament_lint/flake8) y smoke tests de launch.
- Añadir pruebas específicas para teleop y bridges.

---

### I. ASPECTOS DE SEGURIDAD Y OPERACIÓN

#### ✅ Watchdog en teleop
- `joy_teleop.py` implementa timeout (0.5 s default).
- Publica comandos cero si no hay joy.

#### ⚠️ **Sin E-Stop Hardware**
- Simulación OK, pero si hay hardware físico (futuro):
  - Falta circuito e-stop de emergencia.
  - Sin diode de latching.
  - Recomendación: Agregar servicio ROS2 `emergency_stop` que pare todos los controladores.

#### ⚠️ **Root en Docker**
- Container corre como root (sin USER).
- **RIESGO:** Si container es comprometido, acceso full a host.
- **SOLUCIÓN:** Crear usuario `ros` con HOME, cambiar en docker-compose.

#### ✅ Red aislada
- Docker compose usa red `rosnet` propia (no expone interfaces del host innecesariamente).

---

## III. PROBLEMAS CRÍTICOS

| ID | Severidad | Componente | Problema | Impacto | Solución | Estado |
|----|-----------|-----------|----------|---------|----------|--------|
| **C1** | CRÍTICA | Scripts | `joint_state_aggregator.py` no es ejecutable | Falla en runtime al intentar `ros2 run mm_bringup joint_state_aggregator.py` | `chmod +x` o entry_points en CMake | ✅ Resuelto |
| **C2** | CRÍTICA | Config | `base_controllers.yaml` desincronizado con XACRO scale | Si scale ≠ 1.0, mecánica incorrecta (odometría rota) | Template Jinja2 o parámetro dinámico | ⚠️ Pendiente |
| **C3** | CRÍTICA | Launch | Arranque de Gazebo sin espera de servicio (estado previo) | Frágil en sistemas lentos; Gazebo puede no estar listo | Usar `wait-for-service` con timeout configurable | ✅ Resuelto |
| **C4** | CRÍTICA | DevOps | Sin CI/CD (GitHub Actions) | Cambios no validados; incompatibilidades ocultas | Crear `.github/workflows/ci.yaml` | ✅ Resuelto |
| **C5** | CRÍTICA | Config | SRDF no parametrizado (prefix/scale estáticos) | Si cambias `arm_prefix:=custom_`, MoveIt falla | Generar SRDF desde Xacro o template | ⚠️ Pendiente |

---

## IV. PROBLEMAS MAYORES

| ID | Severidad | Componente | Problema | Impacto | Solución |
|----|-----------|-----------|----------|---------|----------|
| **M1** | MAYOR | Launch | Xacro sin `--inorder` | Potencial orden incorrecto de macros | Usar `['xacro', '--inorder', ...]` |
| **M2** | MAYOR | Docs | Sin versiones exactas de dependencias | Incompatibilidad con futuras versiones ROS | Pin versions en package.xml (ej: `ros2_control (>=2.25,<3.0)`) |
| **M3** | MAYOR | Config | Jointnames hardcodeados en YAML (joy_teleop.yaml, arm_controllers.yaml) | Si cambias URDF, YAML sigue viejo | Extraer joint names desde URDF en runtime o generador de config |
| **M4** | MAYOR | Docker | Root user en container | Riesgo de seguridad si container comprometido | Crear usuario `ros` |
| **M5** | MAYOR | Testing | Sin launch_testing ni unit tests | Cambios rompen funcionalidad sin aviso | Agregar tests básicos (smoke test de launch) |

---

## V. PROBLEMAS MENORES

| ID | Severidad | Componente | Problema | Impacto | Solución |
|----|-----------|-----------|----------|---------|----------|
| **m1** | MENOR | Launch | `display.launch.py` genera en `/tmp/` (temporal) | Si múltiples instancias, pueden colisionar | Usar `model_cache_dir` (parámetro) también en display |
| **m2** | MENOR | Docs | AGENTS.md está fuera de lugar | Confusión sobre propósito del archivo | Mover a CONTRIBUTING.md o fusionar con README |
| **m3** | MENOR | Config | Sin comentarios en YAML explicando parámetros | Usuarios no saben qué ajustar | Agregar comentarios en YAML (ej: `# Max vel base [m/s]`) |
| **m4** | MENOR | Docker | Sin HEALTHCHECK | Difficl detectar si servicios fallaron | Agregar `HEALTHCHECK CMD curl localhost:8080` |
| **m5** | MENOR | Code | Algunas expresiones `PythonExpression` son largas | Difícil de leer | Refactorizar a variables intermedias o funciones helper |

---

## VI. RECOMENDACIONES PRIORITARIAS (ROADMAP)

### INMEDIATO (Semana 1)
- [x] **C1:** `chmod +x ros2_ws/src/mm_bringup/scripts/joint_state_aggregator.py`
- [x] **C4:** Crear `.github/workflows/ci.yaml` (colcon build + test)
- [x] **M1:** Cambiar `'xacro'` a `['xacro', '--inorder']` en launch files
- [ ] **M4:** Crear usuario `ros` en Dockerfile, actualizar docker-compose

### CORTO PLAZO (Semana 2-3)
- [ ] **C2:** Resolver desincronización scale en controllers
  - Opción A: Generar YAML desde Jinja2 template
  - Opción B: Hacer `base_controllers` dinámico usando parámetros de launch
- [x] **C3:** Reemplazar `TimerAction(5.0)` con espera real de servicios
- [ ] **C5:** Parametrizar SRDF o generarlo desde Xacro
- [ ] **M2:** Pin versions en `package.xml` (ej: `ros2_control (>=2.25,<3.0)`)
- [ ] Agregar `launch_testing` básico (smoke test de modes.launch.py)
- [ ] Completar `info.txt` con requirements exactos

### MEDIANO PLAZO (Mes 1)
- [ ] **M3:** Extraer joint names dinámicamente (lectura URDF en runtime)
- [x] Crear CONTRIBUTING.md y CHANGELOG.md
- [ ] Agregar sección de troubleshooting expandida
- [ ] Crear entry points en CMakeLists para scripts (mejor que PROGRAMS)
- [ ] Sphinx docs con API reference

### LARGO PLAZO (Mes 2+)
- [ ] Sistema de tests de integración (gazebo + joy + arm movement)
- [ ] Documento de SIL/HIL testing para hardware físico
- [ ] Versionado semántico (v0.2.0, v1.0.0, etc.)

---

## VII. CHECKLIST DE CALIDAD

- [x] **Código limpio:** Sí
- [x] **Documentación presente:** Sí (excepto versiones exactas)
- [ ] **Tests automatizados:** NO (CRÍTICO)
- [x] **CI/CD:** Sí (básico, build + test)
- [ ] **Versionado:** Parcial (CHANGELOG presente, faltan tags git)
- [ ] **Dependencias pinned:** NO (sin versions exactas)
- [x] **Seguridad básica:** Parcial (watchdog OK, root en Docker malo)
- [x] **Multi-plataforma:** Sí (Docker multi-arch)

---

## VIII. COMPARATIVA CON PROYECTOS DE REFERENCIA (Spot/Stretch/Fetch)

**Patrones comunes observados en repositorios de manipuladores móviles (ROS 2):**

1. **Scripts ejecutables (C1):**
   - Usan `ament_python` con `console_scripts` o `install(PROGRAMS ...)` y permisos +x.
2. **Parámetros de base sin `scale` (C2):**
   - Evitan `scale` en la base y declaran constantes físicas; si se necesita escalado, generan YAML con plantillas o pasan parámetros desde launch.
3. **Arranque de Gazebo sin delays fijos (C3):**
   - Esperan servicios `/clock` o `/world/<world>/control` antes de spawnear controladores.
4. **CI mínimo obligatorio (C4):**
   - GitHub Actions con `colcon build`, linters (`ament_lint_auto`) y smoke tests de launch.
5. **MoveIt con SRDF generado (C5):**
   - SRDF generado por MoveIt Setup Assistant; para multi-robot se usa SRDF por robot o SRDF parametrizable.

**Fuentes oficiales recomendadas:**
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/
- MoveIt 2: https://moveit.picknik.ai/
- Nav2: https://navigation.ros.org/

---

## IX. CONCLUSIÓN

El proyecto **ros2-sim-vnc** tiene una **arquitectura sólida y bien documentada**, apropiada para un sistema de teleoperación educativo o de investigación. Aún quedan **pendientes críticos** antes de producción:

1. Config desincronizado con escala dinámica (C2).
2. Sin automated tests reales (solo CI básico).
3. SRDF sin parámetros (C5).

**Próximos pasos recomendados:**
- Resolver los 5 problemas críticos en el sprint actual.
- Mantener y extender CI (lint + launch_testing).
- Versionar el código (git tags, CHANGELOG).
- Documentar requirements exactos.

**Puntaje Final:** 7/10 (arquitectura excelente, DevOps deficiente)

---

## X. ACTUALIZACIÓN DEL REPORTE (estado real del repositorio)

**Estado:** Actualización parcial enfocada en primera entrega (simulación + teleop).

### Estado de problemas críticos

| ID | Problema | Estado actual | Nota |
|----|----------|--------------|------|
| C1 | `joint_state_aggregator.py` no ejecutable | ✅ Resuelto | Script ahora es ejecutable. |
| C2 | Controllers desincronizados con scale | ⚠️ Pendiente | Se recomienda mantener `base_scale=1.0` o generar YAML dinámico. |
| C3 | Arranque de Gazebo sin espera de servicio | ✅ Resuelto | Espera `/world/<world_name>/control` con timeout. |
| C4 | Sin CI/CD | ✅ Resuelto | Workflow CI creado en `.github/workflows/ci.yaml`. |
| C5 | SRDF no parametrizado | ❌ Pendiente | MoveIt usa SRDF fijo; válido para un solo brazo. |

### Cambios aplicados relevantes

1. `xacro --inorder` usado en sim/display para evitar errores de orden.
2. Espera de servicio `/world/<world_name>/control` antes de spawnear controladores.
3. CI básico con GitHub Actions (`.github/workflows/ci.yaml`).
4. Agregador de `/joint_states` para base+brazo.
5. Mundo simplificado (plano + cubo) para reducir costo de simulación.
6. Metadata de paquetes (`package.xml`) completada.

### Próximas prioridades (alineadas a proyectos de referencia)

1. **C2:** Evitar `scale` en la base o generar `base_controllers.yaml` dinámico.
2. **C5:** Generar SRDF con MoveIt Setup Assistant y fijar prefijos estables.
3. Añadir `launch_testing` mínimo para `modes.launch.py`.

---

**Última actualización:** 2025-12-27 (revisión manual).