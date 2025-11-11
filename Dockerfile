FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash","-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# Preparar dependencias basicas para ROS y vision
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions python3-colcon-ros \
    python3-opencv python3-numpy python3-serial python3-matplotlib \
    python3-pil python3-av python3-yaml \
    ros-jazzy-cv-bridge ros-jazzy-image-transport \
    ffmpeg libgl1 libglib2.0-0 \
    iproute2 net-tools iputils-ping curl nano vim tree \
    xauth x11-apps \
 && rm -rf /var/lib/apt/lists/*

# Configurar pip del sistema sin cache
ENV PIP_NO_CACHE_DIR=1
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Instalar el cliente Tello sin dependencias redundantes
RUN python3 -m pip install --no-deps djitellopy==2.5.0

# Definir workspace ROS2
WORKDIR /root/ros2_ws
RUN mkdir -p src
COPY ./src /root/ros2_ws/src

# Construir paquetes con colcon
RUN . /opt/ros/jazzy/setup.sh && colcon build

WORKDIR /root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
