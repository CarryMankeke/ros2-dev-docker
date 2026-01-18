# ROADMAP
Author: Camilo Soto Villegas
Contact: camilo.soto.v@usach.cl
Project: clean_v2
Target: ROS 2 Jazzy + Gazebo Harmonic

This roadmap follows the core vs opt-in pattern. Core stays stable; opt-in features are enabled by launch args and must not break core.

## Phase 0: Core health and reproducible scripts
Goal: Ensure the core stack is stable and validated by scripts.
Changes:
- Add host scripts under clean_v2/scripts to run core and smoke tests.
- Keep core_health_check as the gate (SIM_TIME/TF/CONTROLLERS PASS).
Feature flags:
- use_rviz:=true (core verify view only)
- use_teleop:=false, use_nav2:=false, use_moveit:=false, use_rqt:=false
Tests:
- scripts/core_health.sh
- scripts/smoke/smoke_sim_basic.sh
Done:
- Core gate passes on mm1.
- Smoke sim basic passes with sensors WARN only when expected.
Risks:
- Startup timing causes sensor WARN. Mitigation: retry/backoff in checks.

## Phase 1: Sim basic stable
Goal: Spawn robot in Gazebo, stable TF, and base sensors publishing.
Changes:
- URDF sensors in mm_robot.urdf.xacro and bridges in sim_mm.launch.py.
- Ensure /scan and base cameras publish in sim_mm.
Feature flags:
- enable_lidar:=true, enable_imu:=true, enable_ee_imu:=false
Tests:
- scripts/smoke/smoke_sim_basic.sh
Done:
- /mm1/scan and base cameras publish at least one message.
- core_health_check PASS (SIM_TIME/TF/CONTROLLERS).
Risks:
- Gazebo sensor system missing in world. Mitigation: ensure sensors system plugin enabled.

## Phase 2: Local EKF opt-in
Goal: Provide robot_localization EKF for odom fusion (encoders + IMU).
Changes:
- ekf.launch.py and ekf.yaml.in under mm_bringup.
- ekf_optin_check.py for opt-in validation.
Feature flags:
- use_ekf:=false by default.
Tests:
- scripts/smoke/smoke_ekf_local.sh
Done:
- EKF node running and /odometry/filtered or configured topic published.
- Core gate still passes without EKF.
Risks:
- TF conflict if publish_tf=true. Mitigation: keep publish_tf false unless core TF is disabled.

## Phase 3: Global localization opt-in
Goal: Provide map->odom via AMCL or SLAM.
Changes:
- nav2_min.launch.py and nav2_params.yaml.in.
- Optional slam_toolbox launch (future).
Feature flags:
- use_nav2:=false by default.
Tests:
- scripts/smoke/smoke_nav2.sh
Done:
- Nav2 lifecycle nodes active.
- map->odom available when AMCL/SLAM is running.
Risks:
- Missing /initialpose in AMCL. Mitigation: document initialpose in RViz or CLI.

## Phase 4: Nav2 navigation opt-in
Goal: Autonomous navigation with cmd_vel pipeline.
Changes:
- Nav2 controller and planner config tuned for omni base.
- cmd_vel mux path defined (nav2 -> cmd_vel_nav2 -> mux -> controller).
Feature flags:
- use_mux:=false by default; use_nav2:=false by default.
Tests:
- scripts/smoke/smoke_nav2.sh
Done:
- navigate_to_pose action available; cmd_vel pipeline exists.
Risks:
- cmd_vel conflicts with teleop. Mitigation: mux priorities.

## Phase 5: MoveIt opt-in
Goal: Planning and execution for arm + gripper in sim.
Changes:
- mm_moveit_config SRDF aligned with URDF.
- moveit_core_integration_check.sh gate.
Feature flags:
- use_moveit:=false by default.
Tests:
- scripts/smoke/smoke_moveit.sh
Done:
- move_group active and FJT actions visible under /mm1.
Risks:
- Controllers not running. Mitigation: start core before MoveIt gate.

## Phase 6: Pick and place demo
Goal: Simple pick and place in sim with a fixed object.
Changes:
- Minimal scene object and demo script.
- Optional perception input (future).
Feature flags:
- use_moveit:=true; demo is opt-in.
Tests:
- smoke_moveit + demo script log check.
Done:
- Trajectory executes without errors.
Risks:
- Scene not synchronized. Mitigation: planning scene update on launch.

## Phase 7: Multi-robot
Goal: Run mm1 and mm2 together without collisions.
Changes:
- sim_mm_dual.launch.py and namespaced configs.
Feature flags:
- enable_dual:=false by default.
Tests:
- scripts/smoke/smoke_multirobot.sh
Done:
- Both robots pass core gate and no topic or frame collisions.
Risks:
- Duplicate /clock or bridge nodes. Mitigation: strict namespace and single sim.
