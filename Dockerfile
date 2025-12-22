# Base oficial (Docker Official Image) multi-arch: arm64 OK en Mac M1
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0 \
    # Para Mac+Docker (sin GPU): forzar OpenGL por software
    LIBGL_ALWAYS_SOFTWARE=1 \
    # A veces ayuda a Qt (RViz) en entornos virtuales
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y --no-install-recommends \
    # --- GUI + escritorio remoto (VNC/noVNC) ---
    xfce4 xfce4-terminal dbus-x11 x11-utils \
    xvfb x11vnc novnc websockify supervisor \
    # utilidades mínimas
    curl ca-certificates git \
    # --- ROS GUI completo (incluye RViz2) ---
    ros-jazzy-desktop-full \
    # --- Gazebo Harmonic + integración ROS↔Gazebo (ros_gz) ---
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim-demos \
    ros-jazzy-ros-gz-image \
    # --- ros2_control + controladores (para diff_drive, etc.) ---
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    # --- MoveIt2 (planificacion y OMPL) ---
    ros-jazzy-moveit \
    # (opcional pero útil para control por teclado)
    ros-jazzy-teleop-twist-keyboard \
    # Extras
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
 && apt-get autoremove -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

EXPOSE 8080
CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]
