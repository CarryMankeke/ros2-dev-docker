# Pseudocódigo de empaquetado para mm_base_description

"""pseudo
proyecto mm_base_description:
  crear:
    - declarar cmake_minimum_required(3.8)
    - registrar ament_cmake y xacro
  leer:
    - carpeta urdf con modelo de base omnidireccional
  actualizar:
    - instalar urdf -> share/mm_base_description
  borrar:
    - limpiar artefactos de build/install/log
"""
