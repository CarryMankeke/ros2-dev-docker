cmake_minimum_required(VERSION 3.8)
project(mm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_frame_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
