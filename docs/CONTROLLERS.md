# CONTROLLERS
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## Alcance
Contrato formal de controladores y controller_manager por robot. Se valida por namespace y estado (active).

## controller_manager por robot
1. mm1:
   - `/mm1/controller_manager`
2. mm2:
   - `/mm2/controller_manager`

## Controladores esperados
### Base
1. `joint_state_broadcaster`
   - Joints: todos (solo estado)
2. `omni_wheel_controller`
   - Joints: `__PREFIX__front_left_wheel_joint`
   - Joints: `__PREFIX__front_right_wheel_joint`
   - Joints: `__PREFIX__rear_left_wheel_joint`
   - Joints: `__PREFIX__rear_right_wheel_joint`

### Brazo + gripper
1. `arm_trajectory_controller`
   - Joints: `__PREFIX__arm_shoulder_pan_joint`
   - Joints: `__PREFIX__arm_shoulder_lift_joint`
   - Joints: `__PREFIX__arm_elbow_joint`
   - Joints: `__PREFIX__arm_wrist_1_joint`
   - Joints: `__PREFIX__arm_wrist_2_joint`
   - Joints: `__PREFIX__arm_wrist_3_joint`
2. `gripper_trajectory_controller`
   - Joints: `__PREFIX__gripper_joint`

## Estados esperados
1. Todos los controladores anteriores deben estar en estado `active`.

## Validacion (ROS 2)
1. Listar controladores (mm1):
   - `ros2 service call /mm1/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}`
2. Validar estado (mm1):
   - Buscar `state: active` para cada controlador esperado.
3. Repetir para mm2:
   - `ros2 service call /mm2/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}`

## Sim vs real
1. Los controladores son agnosticos a hardware (sim o real).
2. En sim, `gz_ros2_control` proporciona el hardware simulado.
