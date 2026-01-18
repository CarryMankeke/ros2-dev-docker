# Nav2 base para mm_bringup
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Objetivo
1. Preparar una configuracion base de Nav2 para una base omni.
2. Mantener sincronizacion con `/clock` y `use_sim_time`.

## Pasos
1. Compilar el workspace:
   - `cd clean_v2/ros2_ws && colcon build --symlink-install`
2. Lanzar simulacion con LiDAR:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_ enable_lidar:=true`
3. Lanzar Nav2 (SLAM por defecto):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup nav2_min.launch.py namespace:=mm1 prefix:=mm1_ slam:=true`

## Notas
- El LiDAR publica en `/<namespace>/scan` y se puentea con `ros_gz_bridge`.
- Se configura QoS explicito para `/scan` (best_effort, volatile, keep_last, depth 10) en los costmaps para evitar bloqueos por datos de sensores.
- Ajusta velocidades y limites en `nav2_params.yaml.in` segun dinamica real de la base.

## Configuracion Especifica para Base Omnidireccional

### 1. Footprint del Robot (OBLIGATORIO)
Para la base rectangular 0.50m × 0.60m, **DEBES** definir el footprint en los costmaps:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
      # NO uses robot_radius para robots no circulares

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
```

**Razon**: Sin footprint, Nav2 asume robot circular y puede causar colisiones en esquinas.

### 2. AMCL para Robot Omnidireccional

```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"  # NO DifferentialMotionModel

    # Parametros alpha optimizados para omnidireccional:
    alpha1: 0.2   # Rotacion por rotacion
    alpha2: 0.2   # Rotacion por traslacion
    alpha3: 0.3   # Traslacion por traslacion (aumentar para omni)
    alpha4: 0.2   # Traslacion por rotacion
    alpha5: 0.3   # Ruido lateral (CRITICO para omni - aumentar de 0.1)
```

### 3. DWB Controller (Holonomico)

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      holonomic_robot: true  # IMPORTANTE para omnidireccional

      # Velocidades omnidireccionales
      min_vel_x: -0.3
      max_vel_x: 0.6
      min_vel_y: -0.3  # Movimiento lateral
      max_vel_y: 0.3
      max_vel_theta: 1.0

      min_speed_xy: 0.0  # NO 0.5 (eso es para differential)
      max_speed_xy: 0.7
```

### 4. Parametros del Controlador Base

Asegurate que en `mm_controllers.yaml.in`:

```yaml
omni_wheel_controller:
  ros__parameters:
    robot_radius: 0.357  # Distancia centro->rueda (NO 0.305)
    wheel_radius: 0.07   # Radio de rueda
    wheel_offset: 0.0    # Offset angular primera rueda
```

**Calculo de robot_radius**:
```
wheel_x = 0.263m (posicion x de rueda desde centro)
wheel_y = 0.241m (posicion y de rueda desde centro)
robot_radius = sqrt(wheel_x² + wheel_y²) = sqrt(0.263² + 0.241²) ≈ 0.357m
```

## Seguridad
- Agrega parada de emergencia y watchdog de `cmd_vel` antes de operar el robot real.
- Limita velocidades maximas en Nav2 y en el controlador base para evitar movimientos inesperados.
- El twist_mux (opt-in) proporciona arbitracion segura entre teleop y nav2.

## Mejores Practicas
Ver **docs/BEST_PRACTICES_FROM_PUBLIC_REPOS.md** seccion 3 para detalles completos sobre configuracion Nav2 para robots omnidireccionales basados en repos publicos de ROS2 Jazzy.

## Referencias
- Nav2: https://navigation.ros.org/
- Nav2 Tuning Guide: https://docs.nav2.org/tuning/index.html
- AMCL Configuration: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/index.html
