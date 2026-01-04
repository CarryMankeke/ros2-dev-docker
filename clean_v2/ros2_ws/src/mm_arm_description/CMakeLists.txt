cmake_minimum_required(VERSION 3.8)
project(mm_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
