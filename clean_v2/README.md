# clean_v2

Base limpia para un manipulador movil (base omni + brazo) con soporte multi-robot (gemelos `mm1` y `mm2`).

## Objetivo
- Construir desde cero una estructura estable y escalable.
- Validar por etapas: base -> brazo + gripper -> MoveIt 2 -> Nav2 -> sensores.
- Mantener compatibilidad con ROS 2 Jazzy + Gazebo Harmonic.

## Estructura
- `ros2_ws/`: workspace ROS 2.
- `docs/`: documentacion tecnica y guias de pruebas.

## Ejecucion minima
1. Construir:
   - `cd clean_v2/ros2_ws && colcon build --symlink-install`
2. Lanzar base (sin Gazebo):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup base_min.launch.py namespace:=mm1 prefix:=mm1_`

3. Lanzar simulacion minima (Gazebo):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_min.launch.py namespace:=mm1 prefix:=mm1_`
