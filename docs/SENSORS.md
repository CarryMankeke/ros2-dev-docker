# SENSORS: camaras y convenciones
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Alcance
Este documento describe la integracion de camaras RGB en el manipulador movil (mm) para simulacion con Gazebo Harmonic y ROS 2 Jazzy. El objetivo es tener un solo robot runtime, un solo arbol TF y sensores namespaced.

## Convenciones de frames
1. Base:
   - `__PREFIX__cam_front_link`
   - `__PREFIX__cam_left_link`
   - `__PREFIX__cam_right_link`
   - `__PREFIX__cam_rear_link`
2. End-effector:
   - `__PREFIX__ee_cam_mount_link`
   - `__PREFIX__ee_cam_link`
3. Cadena TF (sin dobles padres):
   - `odom -> base_footprint -> base_link -> cam_*_link`
   - `tool0 -> ee_cam_mount_link -> ee_cam_link`

## Topicos de camaras (ROS)
1. Imagen:
   - `/<ns>/camera/front/image`
   - `/<ns>/camera/left/image`
   - `/<ns>/camera/right/image`
   - `/<ns>/camera/rear/image`
   - `/<ns>/camera/ee/image`
2. CameraInfo:
   - `/<ns>/camera/front/camera_info`
   - `/<ns>/camera/left/camera_info`
   - `/<ns>/camera/right/camera_info`
   - `/<ns>/camera/rear/camera_info`
   - `/<ns>/camera/ee/camera_info`

## frame_id esperado
1. Base:
   - `/<ns>/camera/front/*` -> `__PREFIX__cam_front_link`
   - `/<ns>/camera/left/*` -> `__PREFIX__cam_left_link`
   - `/<ns>/camera/right/*` -> `__PREFIX__cam_right_link`
   - `/<ns>/camera/rear/*` -> `__PREFIX__cam_rear_link`
2. End-effector:
   - `/<ns>/camera/ee/*` -> `__PREFIX__ee_cam_link`

## QoS esperado
1. Imagen y CameraInfo:
   - Reliability: `best_effort`
   - History: `keep_last`
   - Depth: `5` (o mayor)

## Parametros de pose (xacro)
Todos los offsets se declaran como argumentos xacro en `mm_robot.urdf.xacro` y se propagan a `mm_base_macro.xacro` y `mm_arm_macro.xacro`.

Base (ejemplo de defaults):
1. `cam_front_x/y/z`, `cam_front_roll/pitch/yaw`
2. `cam_left_x/y/z`, `cam_left_roll/pitch/yaw`
3. `cam_right_x/y/z`, `cam_right_roll/pitch/yaw`
4. `cam_rear_x/y/z`, `cam_rear_roll/pitch/yaw`

EE (ejemplo de defaults):
1. `ee_cam_mount_x/y/z`, `ee_cam_mount_roll/pitch/yaw`
2. `ee_cam_x/y/z`, `ee_cam_roll/pitch/yaw`

## Bridging Gazebo -> ROS
1. Las camaras publican en Gazebo con sufijo `image_raw` (definido en URDF).
2. `ros_gz_bridge` publica:
   - `/<ns>/camera/<name>/image_raw`
   - `/<ns>/camera/<name>/camera_info_raw` (via remap del bridge)
3. `mm_bringup/scripts/camera_frame_republisher.py` re-publica a `image` y `camera_info` corrigiendo `frame_id`.

## Sim vs real
1. Sim-only:
   - Sensores definidos en Gazebo (camera plugins).
   - `ros_gz_bridge` para imagenes y camera_info.
   - `camera_frame_republisher.py`.
2. Real:
   - Drivers de camara publican directamente `/<ns>/camera/*`.

## Validacion minima (sim_time)
1. Verificar TF de camaras:
   - `ros2 run tf2_ros tf2_echo <prefix>base_link <prefix>cam_front_link`
   - `ros2 run tf2_ros tf2_echo <prefix>tool0 <prefix>ee_cam_link`
2. Verificar imagenes:
   - `ros2 topic echo /<ns>/camera/front/image --once`
   - `ros2 topic echo /<ns>/camera/ee/image --once`
3. Verificar CameraInfo:
   - `ros2 topic echo /<ns>/camera/front/camera_info --once`

## Referencias oficiales
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic
