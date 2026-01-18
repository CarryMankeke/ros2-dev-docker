# UPSTREAM REFERENCES
Author: Camilo Soto Villegas
Contact: camilo.soto.v@usach.cl
Project: clean_v2
Target: ROS 2 Jazzy + Gazebo Harmonic

This file lists upstream public references used as guidance for structure, launch patterns, and opt-in integration. These are references only, not a contract.

- ROS 2 Jazzy documentation
  - Link: https://docs.ros.org/en/jazzy/
  - Use: launch conventions, parameters, QoS, and package layout.

- Gazebo Harmonic documentation
  - Link: https://gazebosim.org/docs/harmonic/
  - Use: sensors, worlds, and gz sim system plugins (Sensors, Imu).

- ros_gz (ROS <-> Gazebo bridge)
  - Link: https://github.com/gazebosim/ros_gz
  - Use: bridge patterns, topic remap, and namespacing.

- Nav2 (Navigation2)
  - Link: https://github.com/ros-navigation/navigation2
  - Use: bringup structure, lifecycle nodes, controller and planner configs.

- Nav2 docs
  - Link: https://navigation.ros.org/
  - Use: amcl/map server params, cmd_vel pipeline, and RViz plugins.

- robot_localization (EKF)
  - Link: https://github.com/cra-ros-pkg/robot_localization
  - Use: ekf_node params, frame conventions, and two_d_mode usage.

- slam_toolbox
  - Link: https://github.com/SteveMacenski/slam_toolbox
  - Use: SLAM opt-in option for map -> odom in sim.

- MoveIt 2
  - Link: https://github.com/ros-planning/moveit2
  - Use: SRDF/URDF consistency, move_group launch layout, and controller integration.

- ros2_control
  - Link: https://github.com/ros-controls/ros2_control
  - Use: controller manager and joint interface patterns.

- ros2_controllers
  - Link: https://github.com/ros-controls/ros2_controllers
  - Use: joint_trajectory_controller and base controller conventions.
