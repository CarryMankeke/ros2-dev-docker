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
