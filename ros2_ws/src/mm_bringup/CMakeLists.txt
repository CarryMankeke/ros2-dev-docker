# Pseudocódigo de build para mm_bringup
# Estructura CRUD sobre recursos de lanzamiento, configs y scripts.

"""pseudo
proyecto mm_bringup:
  crear:
    - declarar cmake_minimum_required(3.8)
    - habilitar warnings (-Wall -Wextra -Wpedantic)
    - localizar dependencias: ament_cmake
  leer:
    - opciones de lint automáticas (ament_lint_auto) con excepciones temporales
  actualizar:
    - instalar directorios launch, config, worlds, rviz -> share/mm_bringup
    - instalar scripts ejecutables: robot_description_publisher.py, joy_teleop.py
  borrar:
    - limpiar artefactos con colcon clean o rm -rf build/install/log
"""
