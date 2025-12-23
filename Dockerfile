# Pseudocódigo de la imagen ros2-sim-vnc
# No ejecutable; resume los pasos esenciales para construir un contenedor ROS 2 Jazzy con noVNC + Gazebo Harmonic.

# Crear la imagen
# 1) Seleccionar base -> ros:jazzy-ros-base-noble (multi-arquitectura arm64/amd64, usable en macOS M1 y Windows 10 x64 via WSL2)
# 2) Definir variables de entorno: TZ, LANG, DISPLAY, LIBGL_ALWAYS_SOFTWARE, QT_X11_NO_MITSHM, XDG_RUNTIME_DIR
# 3) Instalar paquetes clave:
#    - Entorno gráfico: xfce4, xvfb, x11vnc, novnc, websockify (usar nvidia-container-runtime si hay GPU en Windows)
#    - Herramientas: curl, ca-certificates, git
#    - ROS desktop-full (incluye RViz2) + ros_gz + ros2_control + MoveIt2 (según metapaquetes de https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
#    - Utilidades de teleop (teleop-twist-keyboard) y publicadores de joints
# 4) Limpiar caché de apt para reducir la imagen
# 5) Copiar supervisord.conf al directorio de configuración de supervisor
# 6) Exponer puerto 8080 para acceso noVNC
# 7) Definir comando de entrada: supervisord -c /etc/supervisor/conf.d/supervisord.conf

# Representación estructurada
"""pseudo
imagen ros2-sim-vnc:
  base: ros:jazzy-ros-base-noble
  entorno:
    DISPLAY=:0
    LIBGL_ALWAYS_SOFTWARE=1
    QT_X11_NO_MITSHM=1
    XDG_RUNTIME_DIR=/tmp/runtime-root
  paquetes:
    - xfce4, xvfb, x11vnc, novnc, websockify
    - ros-jazzy-desktop-full
    - ros-jazzy-ros-gz, ros-jazzy-ros-gz-sim-demos, ros-jazzy-ros-gz-image
    - ros-jazzy-ros2-control, ros-jazzy-ros2-controllers, ros-jazzy-gz-ros2-control
    - ros-jazzy-moveit, ros-jazzy-teleop-twist-keyboard
    - ros-jazzy-joint-state-publisher, ros-jazzy-joint-state-publisher-gui
  pasos:
    - apt-get update && upgrade
    - apt-get install paquetes
    - apt-get autoremove && apt-get clean
    - rm -rf /var/lib/apt/lists/*
  runtime:
    copiar supervisord.conf -> /etc/supervisor/conf.d/
    exponer puerto 8080
    comando: supervisord -c /etc/supervisor/conf.d/supervisord.conf
"""
