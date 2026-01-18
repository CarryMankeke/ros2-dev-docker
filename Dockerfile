# Imagen base oficial multi-arquitectura (amd64/arm64)
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0 \
    # En Mac+Docker sin GPU, obliga a usar OpenGL por software
    LIBGL_ALWAYS_SOFTWARE=1 \
    # Qt (RViz) es más estable sin MIT-SHM en entornos virtualizados
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    # --- Entorno gráfico y escritorio remoto (VNC/noVNC) ---
    xfce4 xfce4-terminal dbus-x11 x11-utils \
    xvfb x11vnc novnc websockify supervisor \
    xserver-xorg-core xserver-xorg-video-dummy \
    # Utilidades básicas de sistema
    curl ca-certificates git python3-jinja2 \
    # --- Paquete desktop de ROS (incluye RViz2) ---
    ros-jazzy-desktop-full \
    # --- Gazebo Harmonic + integración ROS-Gazebo (ros_gz) ---
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim-demos \
    ros-jazzy-ros-gz-image \
    # --- ros2_control y controladores para la base móvil ---
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-twist-mux \
    # --- MoveIt2 (planificacion y OMPL) ---
    ros-jazzy-moveit \
    # --- Nav2 + robot_localization (EKF) + RViz plugins ---
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    # Opcional pero útil para teleoperación con teclado
    ros-jazzy-teleop-twist-keyboard \
    # RQt graph para debug opt-in
    ros-jazzy-rqt-graph \
    # Extras de soporte
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
 && apt-get autoremove -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY docker/xorg-dummy.conf /etc/X11/xorg.conf.d/99-dummy.conf

# Crear usuario ros (no-root) para ejecutar servicios
RUN useradd -m -s /bin/bash -G sudo ros \
 && echo "ros:ros" | chpasswd \
 && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Cambiar propiedad de directorios clave a usuario ros
RUN chown -R ros:ros /etc/supervisor/conf.d \
 && mkdir -p /home/ros/.config /home/ros/.cache \
 && chown -R ros:ros /home/ros

EXPOSE 8080
CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]
