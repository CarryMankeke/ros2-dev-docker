cmake_minimum_required(VERSION 3.8)
project(mm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/moveit_optin_check.py
  scripts/moveit_core_integration_check.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
