# TF_CONTRACT
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Alcance
Contrato formal de TF para cada robot (mm1, mm2). TF es global (no namespaced) y los frame_ids deben ser unicos por prefijo.

## Reglas no negociables
1. `odom -> base_footprint` es dinamico (publicado por el controlador de odometria).
2. `base_footprint -> base_link` es estatico (URDF / robot_state_publisher).
3. No puede existir un frame con dos padres.

## Global vs namespaced
1. Global:
   - `/tf` y `/tf_static`
   - `/clock`
2. Namespaced:
   - `/mm1/*`, `/mm2/*` (sensores, controladores, robot_description)

## Arbol TF (por robot)
Origen y tipo por tramo:
1. Controlador (dinamico):
   - `__PREFIX__odom -> __PREFIX__base_footprint`
2. URDF / robot_state_publisher (estatico o por joint_state):
   - `__PREFIX__base_footprint -> __PREFIX__base_link`
   - `__PREFIX__base_link -> __PREFIX__front_left_wheel_link`
   - `__PREFIX__base_link -> __PREFIX__front_right_wheel_link`
   - `__PREFIX__base_link -> __PREFIX__rear_left_wheel_link`
   - `__PREFIX__base_link -> __PREFIX__rear_right_wheel_link`
   - `__PREFIX__base_link -> __PREFIX__lidar_link`
   - `__PREFIX__base_link -> __PREFIX__cam_front_link`
   - `__PREFIX__base_link -> __PREFIX__cam_left_link`
   - `__PREFIX__base_link -> __PREFIX__cam_right_link`
   - `__PREFIX__base_link -> __PREFIX__cam_rear_link`
   - `__PREFIX__base_link -> __PREFIX__arm_root_link`
   - `__PREFIX__arm_root_link -> __PREFIX__arm_base_link`
   - `__PREFIX__arm_base_link -> __PREFIX__arm_link_1`
   - `__PREFIX__arm_link_1 -> __PREFIX__arm_link_2`
   - `__PREFIX__arm_link_2 -> __PREFIX__arm_link_3`
   - `__PREFIX__arm_link_3 -> __PREFIX__arm_link_4`
   - `__PREFIX__arm_link_4 -> __PREFIX__arm_link_5`
   - `__PREFIX__arm_link_5 -> __PREFIX__arm_link_6`
   - `__PREFIX__arm_link_6 -> __PREFIX__tool0`
   - `__PREFIX__tool0 -> __PREFIX__gripper_link`
   - `__PREFIX__tool0 -> __PREFIX__ee_cam_mount_link`
   - `__PREFIX__ee_cam_mount_link -> __PREFIX__ee_cam_link`

## Frames esperados por robot
### mm1 (prefijo mm1_)
- mm1_odom
- mm1_base_footprint
- mm1_base_link
- mm1_front_left_wheel_link
- mm1_front_right_wheel_link
- mm1_rear_left_wheel_link
- mm1_rear_right_wheel_link
- mm1_lidar_link
- mm1_cam_front_link
- mm1_cam_left_link
- mm1_cam_right_link
- mm1_cam_rear_link
- mm1_arm_root_link
- mm1_arm_base_link
- mm1_arm_link_1
- mm1_arm_link_2
- mm1_arm_link_3
- mm1_arm_link_4
- mm1_arm_link_5
- mm1_arm_link_6
- mm1_tool0
- mm1_gripper_link
- mm1_ee_cam_mount_link
- mm1_ee_cam_link

### mm2 (prefijo mm2_)
- mm2_odom
- mm2_base_footprint
- mm2_base_link
- mm2_front_left_wheel_link
- mm2_front_right_wheel_link
- mm2_rear_left_wheel_link
- mm2_rear_right_wheel_link
- mm2_lidar_link
- mm2_cam_front_link
- mm2_cam_left_link
- mm2_cam_right_link
- mm2_cam_rear_link
- mm2_arm_root_link
- mm2_arm_base_link
- mm2_arm_link_1
- mm2_arm_link_2
- mm2_arm_link_3
- mm2_arm_link_4
- mm2_arm_link_5
- mm2_arm_link_6
- mm2_tool0
- mm2_gripper_link
- mm2_ee_cam_mount_link
- mm2_ee_cam_link

## Diagnosticos rapidos
1. Verificar TF critico:
   - `ros2 run tf2_ros tf2_echo mm1_odom mm1_base_footprint`
   - `ros2 run tf2_ros tf2_echo mm1_base_footprint mm1_base_link`
   - `ros2 run tf2_ros tf2_echo mm1_base_link mm1_cam_front_link`
   - `ros2 run tf2_ros tf2_echo mm1_tool0 mm1_ee_cam_link`
