# Quick Start Validation (Docker)
Author: Camilo Soto Villegas
Contact: camilo.soto.v@usach.cl
Project: clean_v2

1) Build inside Docker
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && cd $ROS2_WS && colcon build --symlink-install"

Host shortcuts (optional):
- clean_v2/scripts/core_health.sh --namespace mm1
- clean_v2/scripts/smoke/smoke_sim_basic.sh

2) Launch simulation (Gazebo + RViz)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 launch mm_bringup sim_mm.launch.py headless:=false"

3) Run core health check
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run mm_bringup core_health_check.sh --namespace mm1"

Sensor sources of truth (core):
- Encoders: ros2_control joint_states.
- IMU, LiDAR, cameras in sim: Gazebo sensors in URDF + bridge to ROS 2.
- EE IMU: opt-in (may WARN in health check).

4) Run core health check with active test (optional)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run mm_bringup core_health_check.sh --namespace mm1 --active-test"

5) MoveIt plus core integration gate (opt-in)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run mm_moveit_config moveit_core_integration_check.sh --namespace mm1"

6) EKF opt-in (robot_localization)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 launch mm_bringup ekf.launch.py namespace:=mm1 prefix:=mm1_ use_sim_time:=true"
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run mm_bringup ekf_optin_check.py --namespace mm1"

7) Nav2 opt-in (mm1)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 launch mm_bringup nav2_min.launch.py namespace:=mm1 prefix:=mm1_ use_sim_time:=true"
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run mm_bringup nav2_optin_check.py --namespace mm1"

8) RViz opt-in configs
- Nav2: use mm_bringup/rviz/mm_nav2.rviz.in via nav2_min.launch.py (use_rviz:=true).
- MoveIt: use mm_moveit_config/rviz/mm_moveit.rviz.in via moveit.launch.py (use_rviz:=true).

9) Debug opt-in
- rqt_graph: docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 launch mm_bringup rqt_graph.launch.py use_rqt:=true"

10) Teleop opt-in
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && ros2 launch mm_bringup teleop.launch.py namespace:=mm1 use_teleop:=true"
