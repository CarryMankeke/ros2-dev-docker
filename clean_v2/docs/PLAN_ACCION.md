# Plan de accion (clean_v2)
Autor: Camilo Soto Villegas
Contacto: camilo.soto.v@usach.cl
Proyecto: clean_v2

## 1) Base minima (mm1)
1. URDF/Xacro de base omni (4 ruedas) con prefijo `mm1_`.
2. TF completo: `world -> mm1_base_link` y ruedas.
3. ros2_control en Gazebo (controladores y joint_state_broadcaster).
4. Lanzar Gazebo headless con world minimo.
5. Verificar `/clock`, `/tf`, `/joint_states`.

## 2) Brazo + gripper (mm1)
1. URDF/Xacro del brazo y gripper con prefijo `mm1_`.
2. TF completo hasta efector final.
3. ros2_control para brazo/gripper.
4. Verificar joint_states y TF.

## 3) MoveIt 2 (mm1)
1. SRDF y kinematics.
2. MoveGroup + RViz.
3. Validar planeacion basica.

## 4) Nav2 (mm1)
1. Mapa y parametros.
2. Localizacion y planificacion.
3. Validar velocidad y odometria.

## 5) Sensores (mm1)
1. Lidar, camaras e IMU.
2. QoS explicito en topicos criticos.
3. Verificar sincronizacion con `/clock`.

## 6) Gemelo (mm2)
1. Repetir el stack con prefijo `mm2_` y namespace `mm2`.
2. Verificar que no haya colisiones de TF o topics.

## Seguridad
- Incluir watchdogs y parada de emergencia en nodos de control.
- Documentar parametros criticos y limites de velocidad.

## Fuentes
- ROS 2 Jazzy (documentacion oficial)
- Gazebo Harmonic (documentacion oficial)
- Nav2 y MoveIt 2 (documentacion oficial)

