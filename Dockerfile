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


ARG MICRO_ROS_AGENT_TAG=3.0.6

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
    # More stuff hahahah :D
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
 && if apt-cache show ros-jazzy-micro-ros-agent > /dev/null 2>&1; then \
      apt-get install -y --no-install-recommends ros-jazzy-micro-ros-agent; \
    else \
      echo "ros-jazzy-micro-ros-agent not available for this arch; building from source." \
      && apt-get install -y --no-install-recommends \
         build-essential cmake \
         libasio-dev libtinyxml2-dev libssl-dev libfoonathan-memory-dev \
         python3-rosdep python3-colcon-common-extensions \
      && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
      && rosdep update \
     && mkdir -p /opt/micro_ros_agent_ws/src \
     && git clone --branch ${MICRO_ROS_AGENT_TAG} --depth 1 https://github.com/micro-ROS/micro-ROS-Agent.git /opt/micro_ros_agent_ws/src/micro_ros_agent \
     && git clone --branch ros2 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/micro_xrce_dds_agent_src \
     && sed -i 's/2\.12\.x/v2.12.2/g' /opt/micro_xrce_dds_agent_src/CMakeLists.txt \
     && grep -q "v2.12.2" /opt/micro_xrce_dds_agent_src/CMakeLists.txt \
     && if ! git -C /opt/micro_xrce_dds_agent_src diff --quiet; then \
          git -C /opt/micro_xrce_dds_agent_src add CMakeLists.txt \
          && git -C /opt/micro_xrce_dds_agent_src -c user.name="codex" -c user.email="codex@local" commit -m "Patch Fast DDS tag for arm64 build"; \
        fi \
     && sed -i 's|https://github.com/eProsima/Micro-XRCE-DDS-Agent.git|file:///opt/micro_xrce_dds_agent_src|' /opt/micro_ros_agent_ws/src/micro_ros_agent/micro_ros_agent/cmake/SuperBuild.cmake \
     && sed -i 's|-DCMAKE_PREFIX_PATH:PATH=<INSTALL_DIR>|-DCMAKE_PREFIX_PATH:PATH=<INSTALL_DIR>;/opt/ros/jazzy|' /opt/micro_ros_agent_ws/src/micro_ros_agent/micro_ros_agent/cmake/SuperBuild.cmake \
      && sed -i 's|-DUAGENT_USE_SYSTEM_FASTDDS:BOOL=ON|-DUAGENT_USE_SYSTEM_FASTDDS:BOOL=OFF|' /opt/micro_ros_agent_ws/src/micro_ros_agent/micro_ros_agent/cmake/SuperBuild.cmake \
      && sed -i 's|-DUAGENT_USE_SYSTEM_FASTCDR:BOOL=ON|-DUAGENT_USE_SYSTEM_FASTCDR:BOOL=OFF|' /opt/micro_ros_agent_ws/src/micro_ros_agent/micro_ros_agent/cmake/SuperBuild.cmake \
      && /bin/bash -lc "source /opt/ros/jazzy/setup.bash \
         && rosdep install --from-paths /opt/micro_ros_agent_ws/src -i -y --rosdistro jazzy" \
      && /bin/bash -lc "source /opt/ros/jazzy/setup.bash \
         && colcon build --symlink-install --base-paths /opt/micro_ros_agent_ws/src \
            --packages-select micro_ros_agent \
            --cmake-args -DMICROROSAGENT_SUPERBUILD=ON" \
      && if [ -f /opt/micro_ros_agent_ws/build/micro_ros_agent/micro_ros_agent ]; then \
           cp /opt/micro_ros_agent_ws/build/micro_ros_agent/micro_ros_agent /usr/local/bin/; \
         fi; \
    fi \
 && apt-get autoremove -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

EXPOSE 8080
CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]
