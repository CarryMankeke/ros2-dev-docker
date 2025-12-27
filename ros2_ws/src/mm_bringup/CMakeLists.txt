cmake_minimum_required(VERSION 3.8)
project(mm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # La siguiente línea omite el linter que valida derechos de autor
  # Bórrala cuando todos los archivos tengan copyright y licencia
  set(ament_cmake_copyright_FOUND TRUE)
  # La siguiente línea desactiva cpplint (solo funciona en repos git)
  # Elimina la excepción cuando el paquete esté en git y todos los archivos tengan licencia
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY launch config worlds rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_description_publisher.py
  scripts/joy_teleop.py
  scripts/joint_state_aggregator.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
