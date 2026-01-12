# Quick Start Validation (Docker)
Author: Camilo Soto Villegas
Contact: camilo.soto.v@usach.cl
Project: clean_v2

1) Build inside Docker
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && cd /home/ros/ros2_ws && colcon build --symlink-install"

2) Launch simulation (Gazebo + RViz)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 launch mm_bringup sim_mm.launch.py headless:=false"

3) Run core health check
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 run mm_bringup core_health_check.sh --namespace mm1"

Sensor sources of truth (core):
- Encoders: ros2_control joint_states.
- IMU, LiDAR, cameras in sim: Gazebo sensors in URDF + bridge to ROS 2.
- EE IMU: opt-in (may WARN in health check).

4) Run core health check with active test (optional)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 run mm_bringup core_health_check.sh --namespace mm1 --active-test"

5) MoveIt plus core integration gate (opt-in)
- docker compose exec ros2-vnc bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 run mm_moveit_config moveit_core_integration_check.sh --namespace mm1"
