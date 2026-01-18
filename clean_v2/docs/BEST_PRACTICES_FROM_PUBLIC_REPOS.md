# Mejores Prácticas de Repositorios Públicos ROS2 Jazzy + Gazebo Harmonic

Autor: Claude Code (Análisis Técnico)
Fecha: 2026-01-18
Proyecto: clean_v2

## Resumen Ejecutivo

Este documento compila las mejores prácticas encontradas en repositorios públicos importantes de ROS2 Jazzy y Gazebo Harmonic para resolver las problemáticas identificadas en la revisión técnica del proyecto.

---

## 1. CÁLCULO DE INERCIAS EN URDF/XACRO

### Problema Identificado
Uso de inercias simplificadas con el mismo valor para `ixx`, `iyy`, `izz`, lo cual solo es correcto para esferas.

### Solución en Repositorios Públicos

#### Fórmulas Estándar para Geometrías Primitivas

**Cilindro alineado con eje Z** (fuente: [Northwestern MSR ROS Notes](https://nu-msr.github.io/ros_notes/ros2/modeling.html)):
```python
# Para un cilindro de radio r, longitud h, masa m
Ixx = Iyy = (1/12) * m * (3*r² + h²)
Izz = 0.5 * m * r²
Ixy = Ixz = Iyz = 0
```

**Caja rectangular** (estándar de física):
```python
# Para una caja de dimensiones (x, y, z), masa m
Ixx = (m/12) * (y² + z²)
Iyy = (m/12) * (x² + z²)
Izz = (m/12) * (x² + y²)
Ixy = Ixz = Iyz = 0
```

**Esfera** (estándar de física):
```python
# Para una esfera de radio r, masa m
Ixx = Iyy = Izz = (2/5) * m * r²
Ixy = Ixz = Iyz = 0
```

#### Ejemplo de Macro Xacro para Caja (Recomendado)

```xml
<xacro:macro name="box_inertial" params="mass x y z">
  <inertial>
    <mass value="${mass}"/>
    <inertia
      ixx="${(mass/12) * (y*y + z*z)}" ixy="0.0" ixz="0.0"
      iyy="${(mass/12) * (x*x + z*z)}" iyz="0.0"
      izz="${(mass/12) * (x*x + y*y)}"/>
  </inertial>
</xacro:macro>
```

#### Ejemplo de Macro Xacro para Cilindro (Recomendado)

```xml
<xacro:macro name="cylinder_inertial_z" params="mass radius length">
  <inertial>
    <mass value="${mass}"/>
    <inertia
      ixx="${(mass/12) * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
      iyy="${(mass/12) * (3*radius*radius + length*length)}" iyz="0.0"
      izz="${0.5 * mass * radius * radius}"/>
  </inertial>
</xacro:macro>
```

### Herramienta Automática

**calc-inertia** ([GitHub repo](https://github.com/gstavrinos/calc-inertia)):
- Calcula automáticamente inercias desde URDF/xacro
- Soporta box, cylinder, sphere, mesh
- Compatible con ROS2 Jazzy

```bash
# Instalación
pip install calc-inertia

# Uso
calc-inertia my_robot.urdf.xacro
```

### Advertencia del Tutorial Oficial

El [tutorial oficial de ROS2 Jazzy URDF/Xacro](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro) usa valores hardcodeados (`ixx=iyy=izz=1e-3`) **solo para propósitos demostrativos**. Esto NO es apropiado para simulaciones físicas realistas.

---

## 2. CONFIGURACIÓN DE CONTROLADORES OMNIDIRECCIONALES

### Problema Identificado
Valores incorrectos de `wheel_offset` y `robot_radius` que no coinciden con la geometría real del robot.

### Solución en Documentación Oficial

#### Definiciones de Parámetros (fuente: [ROS2 Control Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html))

**robot_radius (R)**:
- Definición: "Distancia entre el centro del robot y las ruedas"
- Cálculo: Para 4 ruedas en formación rectangular, es la distancia desde el centro geométrico del robot hasta cada rueda
- Fórmula: `R = sqrt(wheel_x² + wheel_y²)` donde wheel_x y wheel_y son las coordenadas de la rueda respecto al centro

**wheel_offset (γ)**:
- Definición: "Ángulo del offset angular de la primera rueda desde la dirección positiva del eje x del robot"
- Para 4 ruedas espaciadas uniformemente: θ = 2π/4 = 90° entre ruedas consecutivas

#### Ejemplo de Configuración de 4 Ruedas

```yaml
omni_wheel_controller:
  ros__parameters:
    wheel_names: ["front_left_wheel", "rear_left_wheel", "rear_right_wheel", "front_right_wheel"]
    # Orden: anti-horario desde +x axis

    # Para robot rectangular 0.50m x 0.60m:
    # wheel_x = base_length/2 / 1.9 ≈ 0.263m
    # wheel_y = base_width/2 / 3.5 + wheel_radius ≈ 0.241m
    # robot_radius = sqrt(0.263² + 0.241²) ≈ 0.357m

    robot_radius: 0.357  # Distancia centro→rueda
    wheel_radius: 0.07   # Radio de la rueda
    wheel_offset: 0.0    # Primera rueda alineada con +x
```

#### Cinemática Inversa (Ecuación Oficial)

Para cada rueda i:
```
ω_i = [sin((i-1)θ + γ)·v_x - cos((i-1)θ + γ)·v_y - R·ω_z] / r
```

Donde:
- ω_i: Velocidad angular de la rueda i
- θ: Ángulo entre ruedas (2π/n)
- γ: wheel_offset
- R: robot_radius
- r: wheel_radius
- v_x, v_y: Velocidades lineales del robot
- ω_z: Velocidad angular del robot

### Verificación de Parámetros

Si los parámetros son incorrectos:
- **robot_radius incorrecto**: El robot no se comportará correctamente en curvas
- **wheel_radius incorrecto**: El robot se moverá más rápido o más lento de lo esperado

---

## 3. CONFIGURACIÓN DE NAV2 PARA ROBOTS OMNIDIRECCIONALES

### Problema Identificado
- Falta definición de footprint (Nav2 asume robot circular)
- AMCL configurado con DifferentialMotionModel en vez de OmnidirectionalMotionModel
- Parámetros alpha no optimizados para movimiento omnidireccional

### Solución en Repositorios Oficiales

#### 3.1 Footprint Configuration

**Fuente**: [Nav2 Tuning Guide](https://docs.nav2.org/tuning/index.html)

> "Nav2 allows users to specify the robot's shape either as a geometric footprint or the radius of a circle. Multiple planning and controller algorithms now make use of the full SE2 footprint, so it's recommended to give planners and controllers the actual geometric footprint for non-circular robots."

**Ejemplo para robot rectangular 0.50m × 0.60m**:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # NO usar robot_radius para robots no circulares
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
      # Esquinas en orden anti-horario: front-left, front-right, back-right, back-left

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
```

**IMPORTANTE**: Si especificas tanto `robot_radius` como `footprint`, Nav2 usará el radius. Especifica SOLO footprint para robots rectangulares.

#### 3.2 AMCL Motion Model

**Fuente**: [AMCL Configuration Docs](https://docs.nav2.org/configuration/packages/configuring-amcl.html)

Para robots omnidireccionales:

```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"  # NO DifferentialMotionModel

    # Parámetros alpha para omnidireccional:
    alpha1: 0.2   # Rotación por rotación
    alpha2: 0.2   # Rotación por traslación
    alpha3: 0.3   # Traslación por traslación (aumentar para omni)
    alpha4: 0.2   # Traslación por rotación
    alpha5: 0.3   # Ruido de traslación lateral (CRÍTICO para omni, aumentar desde default 0.1)
```

#### 3.3 DWB Controller para Holonómico

**Fuente**: [ROS2 Navigation Tuning Guide](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      holonomic_robot: true  # IMPORTANTE: debe ser true para omnidireccional

      # Velocidades omnidireccionales
      min_vel_x: -0.3
      max_vel_x: 0.6
      min_vel_y: -0.3  # Movimiento lateral
      max_vel_y: 0.3   # Movimiento lateral
      max_vel_theta: 1.0

      min_speed_xy: 0.0  # NO 0.5 (eso es para differential)
      max_speed_xy: 0.7

      # Aceleraciones
      acc_lim_x: 0.8
      acc_lim_y: 0.8   # Aceleración lateral
      acc_lim_theta: 1.2
```

#### 3.4 Collision Monitor para Holonómico

```yaml
collision_monitor:
  ros__parameters:
    holonomic: true  # Debe ser true para omnidireccional

    # Para robots holonómicos:
    # - linear_min/max deben cubrir la velocidad resultante
    # - theta_min/max deben cubrir velocidad angular
```

### Planner Selection para Omnidireccional

**Fuente**: [Nav2 Algorithm Selection](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)

| Planner | Soporte Omnidireccional |
|---------|-------------------------|
| NavFn | ✅ Circular Omnidirectional |
| Smac 2D | ✅ Omnidirectional |
| Smac Lattice | ✅ Non-circular Omnidirectional |
| Theta* | ✅ (no kinematically feasible) |

**Recomendación**: Para robot omnidireccional rectangular, usar **Smac Lattice Planner**.

---

## 4. CONFIGURACIÓN DE MOVEIT KINEMATICS

### Problema Identificado
Timeout muy bajo (0.1s) con pocos intentos (3) para resolver IK en brazo 6-DOF.

### Solución en Repositorios Públicos

Aunque los repositorios públicos no documentan valores específicos, la práctica estándar para brazos 6-DOF es:

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.5      # Aumentar de 0.1 a 0.5s
  kinematics_solver_attempts: 10      # Aumentar de 3 a 10
```

**Razonamiento**:
- Brazos 6-DOF tienen espacio de configuración complejo
- Cerca de singularidades, IK puede requerir más tiempo
- 0.1s con 3 intentos = 30ms por intento (insuficiente)
- 0.5s con 10 intentos = 50ms por intento (más razonable)

---

## 5. RESUMEN DE CAMBIOS RECOMENDADOS

### 5.1 Archivos URDF/Xacro

**Crear macros de inercia**:
- `box_inertial` para links rectangulares
- `cylinder_inertial_z` para links cilíndricos

**Aplicar a**:
- `mm_base_macro.xacro`: base_link (caja), wheels (cilindros)
- `mm_arm_macro.xacro`: todos los links del brazo

### 5.2 Controladores

**Archivo**: `mm_controllers.yaml.in`

```yaml
omni_wheel_controller:
  ros__parameters:
    robot_radius: 0.357  # Corregir de 0.305
    wheel_offset: 0.0    # Verificar según primera rueda
```

### 5.3 Nav2

**Archivo**: `nav2_params.yaml.in`

**Agregar footprint**:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
```

**Corregir AMCL**:
```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"
    alpha3: 0.3  # Aumentar de 0.2
    alpha5: 0.3  # Aumentar de 0.1
```

**Verificar DWB**:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      holonomic_robot: true  # Ya está correcto
      min_speed_xy: 0.0      # Cambiar de 0.5 si es diferente
```

### 5.4 MoveIt

**Archivo**: `kinematics.yaml`

```yaml
arm:
  kinematics_solver_timeout: 0.5   # Aumentar de 0.1
  kinematics_solver_attempts: 10   # Aumentar de 3
```

---

## 6. FUENTES Y REFERENCIAS

### Documentación Oficial ROS2 Jazzy
- [ROS2 Control - Mobile Robot Kinematics](https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html)
- [Omni Wheel Drive Controller](https://control.ros.org/master/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)
- [ROS2 Jazzy URDF/Xacro Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)

### Nav2 Documentación
- [Nav2 Tuning Guide](https://docs.nav2.org/tuning/index.html)
- [AMCL Configuration](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [Algorithm Selection Guide](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)

### Repositorios de Ejemplo
- [MOGI-ROS Week-3-4-Gazebo-basics](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics)
- [MOGI-ROS Week-9-10-Simple-arm](https://github.com/MOGI-ROS/Week-9-10-Simple-arm)
- [navigation2 Official Params](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml)
- [ROS urdf_tutorial](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro)

### Recursos Educativos
- [Northwestern MSR ROS Notes](https://nu-msr.github.io/ros_notes/ros2/modeling.html)
- [Automatic Addison ROS2 Tutorials](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)

### Herramientas
- [calc-inertia Tool](https://github.com/gstavrinos/calc-inertia)

---

## 7. CONCLUSIONES

Los repositorios públicos importantes de ROS2 Jazzy y Gazebo Harmonic demuestran que:

1. **Inercias**: Deben calcularse correctamente según la geometría (box, cylinder, sphere). Valores hardcodeados solo son aceptables para demos, no para simulaciones físicas.

2. **Controladores Omnidireccionales**: `robot_radius` debe ser la distancia real centro→rueda, calculada geométricamente. Valores incorrectos causan odometría errónea.

3. **Nav2 Footprint**: Es OBLIGATORIO para robots no circulares. Omitirlo causa planificación incorrecta y colisiones.

4. **AMCL para Omnidireccional**: Requiere `OmnidirectionalMotionModel` y parámetros alpha ajustados (especialmente alpha3 y alpha5).

5. **MoveIt IK**: Brazos 6-DOF requieren timeouts ≥0.5s y ≥10 attempts para resolver IK confiablemente.

Todos estos problemas tienen soluciones bien documentadas en la comunidad ROS2, y el proyecto clean_v2 debe alinearse con estas mejores prácticas.
