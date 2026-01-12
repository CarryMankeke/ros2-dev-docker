cmake_minimum_required(VERSION 3.8)
project(mm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_frame_republisher.py
  scripts/rviz_visual_descriptions.py
  scripts/smoke_sim_time.py
  scripts/smoke_tf.py
  scripts/smoke_controllers.py
  scripts/smoke_cameras.py
  scripts/run_smoke_tests.sh
  scripts/core_health_check.py
  scripts/core_health_check.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
