# Plan de accion (clean_v2)
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

> **Ultima actualizacion**: 2026-01-18
> **Estado general**: Todas las fases completadas. Proyecto listo para validacion end-to-end.

## 1) Base minima (mm1) ✅ COMPLETO
1. ✅ URDF/Xacro de base omni (4 ruedas) con prefijo `mm1_`.
2. ✅ TF completo: `mm1_odom -> mm1_base_footprint -> mm1_base_link` y ruedas.
3. ✅ ros2_control en Gazebo (controladores y joint_state_broadcaster).
4. ✅ Lanzar Gazebo headless con world minimo.
5. ✅ Verificar `/clock`, `/tf`, `/joint_states`.

**Validacion**: `ros2 launch mm_bringup sim_min.launch.py namespace:=mm1 prefix:=mm1_`

## 2) Brazo + gripper (mm1) ✅ COMPLETO
1. ✅ URDF/Xacro del brazo y gripper con prefijo `mm1_`.
2. ✅ TF completo hasta efector final (mm1_tool0, mm1_ee_cam_link).
3. ✅ ros2_control para brazo/gripper (arm_trajectory_controller, gripper_trajectory_controller).
4. ✅ Verificar joint_states y TF.

**Validacion**: `ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_`

## 3) MoveIt 2 (mm1) ✅ COMPLETO
1. ✅ SRDF (mm_robot.srdf.xacro) y kinematics (kinematics.yaml).
2. ✅ MoveGroup + RViz via moveit.launch.py.
3. ✅ Validar planeacion basica.

**Validacion**:
```bash
# Terminal 1: Core
ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_

# Terminal 2: MoveIt
ros2 launch mm_moveit_config moveit.launch.py namespace:=mm1 prefix:=mm1_

# Terminal 3: Check
ros2 run mm_moveit_config moveit_core_integration_check.sh --namespace mm1
```

## 4) Nav2 (mm1) ✅ COMPLETO
1. ✅ Parametros renderizados (nav2_params.yaml.in).
2. ✅ Nodos Nav2 (controller_server, planner_server, bt_navigator, behavior_server).
3. ✅ Validar con nav2_optin_check.py.

**Validacion**:
```bash
# Terminal 1: Core
ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_

# Terminal 2: Nav2
ros2 launch mm_bringup nav2_min.launch.py namespace:=mm1 prefix:=mm1_ slam:=true

# Terminal 3: Check
ros2 run mm_bringup nav2_optin_check.py --namespace mm1
```

## 5) Sensores (mm1) ✅ COMPLETO
1. ✅ Lidar (/mm1/scan), camaras base (/mm1/camera/*/image_raw), IMU (/mm1/imu).
2. ✅ QoS explicito en topicos criticos (SensorDataQoS).
3. ✅ Sincronizacion con `/clock` verificada.

**Sensores implementados**:
- LiDAR: /mm1/scan (mm1_lidar_link)
- IMU base: /mm1/imu (mm1_imu_link)
- Camaras base: /mm1/camera/{front,left,right,rear}/image_raw
- Camara EE: /mm1/camera/ee/image_raw (mm1_ee_cam_link)

**Validacion**: `ros2 run mm_bringup core_health_check.py --namespace mm1`

## 6) Gemelo (mm2) ✅ COMPLETO
1. ✅ Stack completo con prefijo `mm2_` y namespace `mm2`.
2. ✅ Sin colisiones de TF o topics (frames unicos mm1_*, mm2_*).

**Validacion**:
```bash
ros2 launch mm_bringup sim_mm_dual.launch.py
ros2 run mm_bringup core_health_check.py --namespace mm1 --check-mm2
```

## Seguridad ✅ IMPLEMENTADO
- ✅ cmd_vel_timeout: 0.5s (watchdog publica cero en ausencia de input).
- ✅ Limites de velocidad en mm_controllers.yaml.in.
- Pendiente: Documentar limites de velocidad y parada de emergencia.

## Scripts de validacion disponibles

| Script | Descripcion |
|--------|-------------|
| core_health_check.py | Validacion completa (TF, controllers, sensors, odom, sim_time) |
| smoke_tf.py | Validacion de TF tree |
| smoke_controllers.py | Validacion de controladores |
| smoke_cameras.py | Validacion de camaras |
| smoke_sim_time.py | Validacion de /clock |
| nav2_optin_check.py | Validacion de nodos Nav2 |
| moveit_core_integration_check.sh | Validacion de MoveIt (move_group y actions) |

## Proximos pasos sugeridos

1. **Limpieza de archivos legacy**:
   - Mover arm_controllers.yaml.in a docs/legacy/ o eliminar
   - mm_arm.urdf.xacro: MANTENER (standalone para pruebas del brazo sin base)
   - mm_arm.srdf.xacro: MANTENER (SRDF standalone para MoveIt solo brazo)

2. **Documentacion adicional**:
   - Documentar limites de velocidad criticos
   - Documentar procedimiento de parada de emergencia

3. **Validacion end-to-end**:
   - Ejecutar todos los smoke tests en secuencia
   - Probar planificacion MoveIt completa
   - Probar navegacion Nav2 con mapa

## Fuentes
- ROS 2 Jazzy (documentacion oficial)
- Gazebo Harmonic (documentacion oficial)
- Nav2 y MoveIt 2 (documentacion oficial)
