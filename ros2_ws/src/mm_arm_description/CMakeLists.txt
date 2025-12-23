# Pseudocódigo de empaquetado para mm_arm_description

"""pseudo
proyecto mm_arm_description:
  crear:
    - declarar cmake_minimum_required(3.8)
    - registrar dependencias ament_cmake y xacro
  leer:
    - carpeta urdf con brazos, gripper y frames tool0/ee_link
  actualizar:
    - instalar urdf -> share/mm_arm_description
  borrar:
    - limpiar artefactos de build/install/log
"""
