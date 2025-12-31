# PLAN DE ACCIÓN INMEDIATO - ros2-sim-vnc
**Prioridad:** 🔴 CRÍTICO  
**Plazo:** Este week  
**Objetivo:** Resolver 3 problemas bloqueadores

---

## 📋 TAREAS

### **TAREA 1: Validación de rutas en launch files (P3)**
**Prioridad:** INMEDIATO  
**Tiempo estimado:** 30 min  
**Impacto:** Evita errores confusos para nuevos usuarios  

#### Problema
```python
# modes.launch.py, sim.launch.py - SIN VALIDACIÓN
nav2_params_file = LaunchConfiguration("nav2_params")
# Si el archivo no existe, falla con error poco claro
```

#### Solución
```python
# Agregar en modes.launch.py
from pathlib import Path

def validate_file_exists(config, filename_desc: str):
    """Valida que un archivo exista y es legible"""
    if not Path(config).exists():
        raise ValueError(
            f"{filename_desc} no encontrado: {config}\n"
            "Verifica ruta y que el archivo exista."
        )

# En OpaqueFunction antes de incluir sublaunchfiles
def _validate_configs(context):
    """Valida rutas de configuración antes de lanzar"""
    
    nav2_params = LaunchConfiguration("nav2_params").perform(context)
    if nav2_params != "":
        validate_file_exists(nav2_params, "nav2_params.yaml")
    
    bridge_params = LaunchConfiguration("bridge_params").perform(context)
    validate_file_exists(bridge_params, "bridge_params.yaml")
    
    joy_teleop_config = LaunchConfiguration("joy_teleop_config").perform(context)
    validate_file_exists(joy_teleop_config, "joy_teleop.yaml")
    
    return []
```

#### Archivos a modificar
- [ ] `ros2_ws/src/mm_bringup/launch/modes.launch.py` (líneas ~80-120)
- [ ] `ros2_ws/src/mm_bringup/launch/sim.launch.py` (líneas ~50-100)

#### Test
```bash
cd ~/Documents/ros2-sim-vnc
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash
source /home/ros/ros2_ws/install/setup.bash

# Test 1: Launch debe fallar limpiamente con ruta incorrecta
ros2 launch mm_bringup modes.launch.py nav2_params:=/invalid/path.yaml 2>&1 | head -20

# Test 2: Launch debe funcionar con parámetros correctos
ros2 launch mm_bringup modes.launch.py --show-args
'
```

---

### **TAREA 2: Generar SRDF dinámicamente desde Xacro (P1)**
**Prioridad:** CRÍTICO  
**Tiempo estimado:** 2-3 hrs  
**Impacto:** MoveIt 2 funcionará con cualquier escala/prefix  

#### Problema Actual
```
mm_arm.urdf.xacro (con scale, prefix) ✅
    ↓ xacro → mm_arm.urdf.xml
    ✅ DINÁMICO
    
mm_arm.srdf (HARDCODEADO)
    joint_1, joint_2, ...
    ❌ NO ESCALA CON arm_prefix / arm_scale
```

#### Solución: Crear `mm_arm.srdf.xacro`

**Paso 1:** Crear archivo template
```bash
cp ros2_ws/src/mm_bringup/config/mm_arm.srdf \
   ros2_ws/src/mm_arm_description/srdf/mm_arm.srdf.xacro
```

**Paso 2:** Editar `mm_arm.srdf.xacro`
```xml
<?xml version="1.0"?>
<robot name="mm_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Argumentos -->
  <xacro:arg name="prefix" default="mm_arm_"/>
  <xacro:property name="prefix" value="$(arg prefix)"/>
  
  <!-- Group para planificación -->
  <group name="arm">
    <joint name="${prefix}joint_1"/>
    <joint name="${prefix}joint_2"/>
    <joint name="${prefix}joint_3"/>
    <joint name="${prefix}joint_4"/>
    <joint name="${prefix}joint_5"/>
    <joint name="${prefix}joint_6"/>
  </group>
  
  <!-- Gripper opcional -->
  <group name="gripper">
    <joint name="${prefix}gripper_joint"/>
  </group>
  
  <!-- Composición -->
  <group name="mm_arm_full">
    <group name="arm"/>
    <group name="gripper"/>
  </group>
  
  <!-- Desabilitar colisiones entre links adyacentes -->
  <disable_collisions link1="${prefix}link_0" link2="${prefix}link_1" reason="Adjacent"/>
  <disable_collisions link1="${prefix}link_1" link2="${prefix}link_2" reason="Adjacent"/>
  <!-- ... más pares ... -->
  
  <!-- End effector -->
  <end_effector name="ee" parent_link="${prefix}link_6" group="gripper"/>
  
</robot>
```

**Paso 3:** Modificar `moveit.launch.py`
```python
# ANTES:
srdf_path = PathJoinSubstitution([
    FindPackageShare("mm_bringup"),
    "config/mm_arm.srdf"
])

# DESPUÉS:
def generate_srdf_file(context):
    """Genera SRDF desde Xacro con parámetros dinámicos"""
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    
    srdf_xacro_file = PathJoinSubstitution([
        FindPackageShare("mm_arm_description"),
        "srdf/mm_arm.srdf.xacro"
    ]).perform(context)
    
    srdf_file = f"/tmp/mm_arm_{arm_prefix.replace('_', '')}.srdf"
    
    # Ejecuta xacro con parámetros
    cmd = [
        "xacro",
        srdf_xacro_file,
        f"prefix:={arm_prefix}"
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro SRDF failed: {result.stderr}")
    
    with open(srdf_file, 'w') as f:
        f.write(result.stdout)
    
    return srdf_file

# En generate_launch_description():
srdf_file = OpaqueFunction(function=generate_srdf_file)

# Usar en Node:
Node(
    package="moveit_ros_move_group",
    executable="move_group",
    parameters=[
        {"robot_description_semantic": Command([
            "cat", srdf_file  # Ahora dinámico
        ])}
    ]
)
```

#### Archivos a crear/modificar
- [ ] Crear: `ros2_ws/src/mm_arm_description/srdf/mm_arm.srdf.xacro`
- [ ] Modificar: `ros2_ws/src/mm_bringup/launch/moveit.launch.py` (~line 40-80)
- [ ] Agregar: `import subprocess` a moveit.launch.py
- [ ] Eliminar: `ros2_ws/src/mm_bringup/config/mm_arm.srdf` (se genera ahora)

#### Test
```bash
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash

# Test generación SRDF con prefix diferente
xacro ros2_ws/src/mm_arm_description/srdf/mm_arm.srdf.xacro prefix:=robot1_arm_

# Test con scale
xacro ros2_ws/src/mm_arm_description/srdf/mm_arm.srdf.xacro prefix:=robot1_arm_ scale:=1.5
'

# Verificar que MoveIt 2 carga correctamente
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash
source /home/ros/ros2_ws/install/setup.bash
ros2 launch mm_bringup modes.launch.py launch_moveit:=true launch_sim:=false launch_rviz:=false 2>&1 | grep -i "srdf\|move_group"
'
```

---

### **TAREA 3: Sincronizar controllers con escala (P2)**
**Prioridad:** CRÍTICO  
**Tiempo estimado:** 1-2 hrs  
**Impacto:** Controllers funcionarán correctamente con cualquier scale  

#### Problema Actual
```yaml
# base_controllers.yaml (HARDCODEADO)
wheel_separation_x: 0.33      # DEBE ser 0.33 * scale
wheel_separation_y: 0.33
wheel_radius: 0.06
```

vs.

```xacro
<!-- mm_base.urdf.xacro (DINÁMICO) -->
<xacro:property name="wheel_separation_x" value="${0.33 * scale}"/>
<xacro:property name="wheel_separation_y" value="${0.33 * scale}"/>
<xacro:property name="wheel_radius" value="${0.06 * scale}"/>
```

#### Solución: Generar controllers.yaml desde template

**Paso 1:** Crear `base_controllers.yaml.jinja2`
```bash
cat > ros2_ws/src/mm_bringup/config/base_controllers.yaml.jinja2 << 'EOF'
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController

joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_name: {{ base_prefix }}front_left_wheel_joint
    front_right_wheel_name: {{ base_prefix }}front_right_wheel_joint
    rear_left_wheel_name: {{ base_prefix }}rear_left_wheel_joint
    rear_right_wheel_name: {{ base_prefix }}rear_right_wheel_joint

    wheel_separation_x: {{ 0.33 * base_scale }}
    wheel_separation_y: {{ 0.33 * base_scale }}
    wheel_radius: {{ 0.06 * base_scale }}

    max_velocity: 1.0
    max_omega: 1.5

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: {{ base_prefix }}base_link
    pose_covariance_diagonal: [0.01, 0.01, 0.01, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
EOF
```

**Paso 2:** Modificar `sim.launch.py` para generar YAML
```python
import yaml
from jinja2 import Template

def _generate_base_controllers_yaml(context):
    """Genera base_controllers.yaml dinámicamente"""
    
    base_prefix = LaunchConfiguration("base_prefix").perform(context)
    base_scale = float(LaunchConfiguration("base_scale").perform(context))
    
    template_file = PathJoinSubstitution([
        FindPackageShare("mm_bringup"),
        "config/base_controllers.yaml.jinja2"
    ]).perform(context)
    
    with open(template_file, 'r') as f:
        template = Template(f.read())
    
    # Renderizar con variables
    rendered = template.render(
        base_prefix=base_prefix,
        base_scale=base_scale
    )
    
    # Guardar en /tmp
    output_file = f"/tmp/base_controllers_{base_prefix.replace('_', '')}.yaml"
    with open(output_file, 'w') as f:
        f.write(rendered)
    
    return output_file

# En generate_launch_description():
base_controllers_yaml = OpaqueFunction(
    function=_generate_base_controllers_yaml,
    on_include=True
)

# Usar en spawner:
Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "mecanum_drive_controller",
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "120",
        "--param-file", base_controllers_yaml  # Ahora dinámico
    ]
)
```

**Paso 3:** Agregar a `sim.launch.py`
```python
# En los imports
from jinja2 import Template
import yaml

# Ya está en OpaqueFunction? Si no:
base_controllers_yaml_node = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "mecanum_drive_controller",
        "--controller-manager", "/controller_manager",
        "--param-file", base_controllers_yaml
    ]
)
```

#### Archivos a crear/modificar
- [ ] Crear: `ros2_ws/src/mm_bringup/config/base_controllers.yaml.jinja2`
- [ ] Crear: `ros2_ws/src/mm_bringup/config/arm_controllers.yaml.jinja2`
- [ ] Modificar: `ros2_ws/src/mm_bringup/launch/sim.launch.py` (~line 150-200)
- [ ] Modificar: `ros2_ws/src/mm_bringup/launch/sim.launch.py` (~line 400-450, agregar OpaqueFunction)
- [ ] Agregar dependencia Jinja2 a `package.xml`: `<exec_depend>jinja2</exec_depend>`

#### Test
```bash
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash
cd /home/ros/ros2_ws
python3 -c "
from jinja2 import Template
import jinja2
print(f\"Jinja2 version: {jinja2.__version__}\")
"
colcon build --symlink-install
'

# Test con scale != 1.0
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash
source /home/ros/ros2_ws/install/setup.bash
ros2 launch mm_bringup modes.launch.py base_scale:=2.0 launch_sim:=true 2>&1 | grep -A 5 "mecanum_drive_controller"
'

# Verifica que los parámetros sean correctos (separación * 2)
docker compose exec -T ros2-vnc bash -lc '
source /opt/ros/jazzy/setup.bash
source /home/ros/ros2_ws/install/setup.bash
ros2 param list | grep "wheel_separation"
ros2 param get /controller_manager mecanum_drive_controller.wheel_separation_x
'
```

---

## 📋 CHECKLIST

```
TAREA 1: Validación de rutas (30 min)
  [ ] Leer TAREA 1 completa
  [ ] Modificar modes.launch.py (OpaqueFunction con validación)
  [ ] Modificar sim.launch.py (validar bridge_params, joy_teleop_config)
  [ ] Test: lanzar con rutas incorrectas → debe fallar con mensaje claro
  [ ] Test: lanzar con rutas correctas → debe funcionar
  [ ] Commit: "fix(launch): add path validation to modes/sim launchfiles"

TAREA 2: SRDF dinámico (2-3 hrs)
  [ ] Leer TAREA 2 completa
  [ ] Crear mm_arm.srdf.xacro desde mm_arm.srdf
  [ ] Modificar moveit.launch.py para generar SRDF
  [ ] Crear package mm_arm_description/srdf/ (if not exists)
  [ ] Test: xacro con prefixes diferentes
  [ ] Test: MoveIt 2 carga correctamente
  [ ] Commit: "refactor(srdf): parametrize srdf generation from xacro"

TAREA 3: Controllers dinámicos (1-2 hrs)
  [ ] Leer TAREA 3 completa
  [ ] Crear base_controllers.yaml.jinja2
  [ ] Crear arm_controllers.yaml.jinja2
  [ ] Modificar sim.launch.py con OpaqueFunction
  [ ] Agregar jinja2 a package.xml
  [ ] Test: generar controllers.yaml con scales diferentes
  [ ] Test: robots con scale:=2.0 se mueven correctamente
  [ ] Commit: "fix(controllers): generate params dynamically from jinja2 templates"

POST-FIXES (Cleanup)
  [ ] colcon build --symlink-install
  [ ] colcon test
  [ ] Actualizar CHANGELOG.md
  [ ] Push a rama feature/fix-critical-issues
```

---

## 🔗 REFERENCIAS

- [PROBLEMAS_ACTUALES.md](PROBLEMAS_ACTUALES.md) - Detalles de los problemas
- [MAPA_PROYECTO.md](MAPA_PROYECTO.md) - Arquitectura general
- [ROS 2 Launch Docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/Launch/Launch-user-guide.html)
- [Jinja2 Templates](https://jinja.palletsprojects.com/en/3.1.x/templates/)
- [Xacro Documentation](https://wiki.ros.org/xacro)

---

## 💬 NOTAS

- Los archivos `.jinja2` usan sintaxis `{{ variable }}` para inyección de valores
- Los `OpaqueFunction` en launch files permiten lógica Python dinámica
- Guardar artefactos en `/tmp` es provisional; considera `~/.cache/mm_bringup` en producción
- Cada `OpaqueFunction` debe retornar una lista de acciones (puede estar vacía)

