# TEST PLAN
Author: Camilo Soto Villegas
Contact: camilo.soto.v@usach.cl
Project: clean_v2
Target: ROS 2 Jazzy + Gazebo Harmonic

This test plan defines what to run when specific files or subsystems change. All commands are run from the host using docker compose exec.

## Core gates (always)
- Core gate command:
  - docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source \\$ROS2_WS/install/setup.bash && ros2 run mm_bringup core_health_check.sh --namespace mm1"
- Expected:
  - SIM_TIME PASS, TF PASS, CONTROLLERS PASS.
  - SENSORS may WARN if data is missing.

## Matrix: file changes -> tests

| Change area | Examples | Required tests |
|---|---|---|
| package.xml or CMakeLists.txt | clean_v2/ros2_ws/src/*/package.xml | colcon build; core_health_check |
| URDF/Xacro/SDF | mm_robot.urdf.xacro, mm_base.urdf.xacro, minimal.world.sdf | smoke_sim_basic.sh; core_health_check |
| Gazebo sensors or bridges | IMU, LiDAR, camera topics | smoke_sim_basic.sh; manual echo of topics |
| Launch files | sim_mm.launch.py, nav2_min.launch.py | smoke_sim_basic.sh; smoke_nav2.sh (if Nav2 touched) |
| Nav2 params | nav2_params.yaml.in | smoke_nav2.sh; core_health_check |
| EKF params | ekf.yaml.in | smoke_ekf_local.sh; core_health_check |
| MoveIt config | mm_robot.srdf.xacro, moveit.launch.py | smoke_moveit.sh; moveit_core_integration_check.sh |
| RViz configs | mm_verify.rviz.in, mm_nav2.rviz.in | Launch opt-in RViz; check config load |
| Teleop or mux | teleop.launch.py, cmd_vel_mux.yaml.in | smoke_sim_basic.sh; manual teleop test |
| Multi-robot | sim_mm_dual.launch.py | smoke_multirobot.sh |

## Script inventory
- clean_v2/scripts/core_health.sh
- clean_v2/scripts/smoke/smoke_sim_basic.sh
- clean_v2/scripts/smoke/smoke_ekf_local.sh
- clean_v2/scripts/smoke/smoke_nav2.sh
- clean_v2/scripts/smoke/smoke_moveit.sh
- clean_v2/scripts/smoke/smoke_multirobot.sh

## Example usage
- Core:
  - clean_v2/scripts/core_health.sh --namespace mm1
- Sim basic:
  - clean_v2/scripts/smoke/smoke_sim_basic.sh
- EKF opt-in:
  - clean_v2/scripts/smoke/smoke_ekf_local.sh
- Nav2 opt-in:
  - clean_v2/scripts/smoke/smoke_nav2.sh
- MoveIt opt-in:
  - clean_v2/scripts/smoke/smoke_moveit.sh
- Multi-robot:
  - clean_v2/scripts/smoke/smoke_multirobot.sh
