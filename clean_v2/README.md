# clean_v2

Base limpia para un manipulador movil (base omni + brazo) con soporte multi-robot (gemelos `mm1` y `mm2`).

## Objetivo
- Construir desde cero una estructura estable y escalable.
- Validar por etapas: base -> brazo + gripper -> MoveIt 2 -> Nav2 -> sensores.
- Mantener compatibilidad con ROS 2 Jazzy + Gazebo Harmonic.

## Estructura
- `ros2_ws/`: workspace ROS 2.
- `docs/`: documentacion tecnica y guias de pruebas.

## Documentacion clave
- `docs/MOVEIT2_BASE.md`
- `docs/NAV2_BASE.md`
- `docs/ROADMAP.md`
- `docs/TEST_PLAN.md`
- `docs/UPSTREAM_REFERENCES.md`

## Ejecucion minima
1. Construir:
   - `cd clean_v2/ros2_ws && colcon build --symlink-install`
2. Lanzar base (sin Gazebo):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup base_min.launch.py namespace:=mm1 prefix:=mm1_`

3. Lanzar simulacion minima (Gazebo):
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_min.launch.py namespace:=mm1 prefix:=mm1_`

4. Lanzar manipulador movil (base + brazo + gripper) en Gazebo:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_mm.launch.py namespace:=mm1 prefix:=mm1_`
   - Verificar controladores:
     - `ros2 control list_controllers --controller-manager /mm1/controller_manager`
   - El brazo y el gripper usan interfaces de posicion (JTC) para integracion con MoveIt 2.

Nota: para evitar saltos de tiempo, ejecuta un solo launch a la vez (un solo `/clock`).

## Scripts de validacion (host)
- `scripts/core_health.sh` (gate del core)
- `scripts/smoke/smoke_sim_basic.sh`
- `scripts/smoke/smoke_ekf_local.sh`
- `scripts/smoke/smoke_nav2.sh`
- `scripts/smoke/smoke_moveit.sh`
- `scripts/smoke/smoke_multirobot.sh`

Notas:
- Teleop y rqt_graph son opt-in. Usa `use_teleop:=true` y `use_rqt:=true` cuando se requiera.

## MoveIt 2 (brazo + gripper)
1. Lanzar MoveIt (solo move_group):
   - `source install/setup.bash`
   - `ros2 launch mm_moveit_config moveit.launch.py namespace:=mm1 prefix:=mm1_`

## Nav2 (base omni)
1. Lanzar Nav2 con SLAM:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup nav2_min.launch.py namespace:=mm1 prefix:=mm1_ slam:=true`

## Multi-robot (mm1 + mm2)
1. Lanzar dos robots en Gazebo:
   - `source install/setup.bash`
   - `ros2 launch mm_bringup sim_mm_dual.launch.py`
