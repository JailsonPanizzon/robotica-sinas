# Imagem ROS com suporte ARM64
FROM ros:humble



ARG WORKSPACE_ROOT="/ros2"
ARG PACKAGE_NAME="ros2"

RUN echo "Building on architecture: $(uname -m)"



# Workspace
WORKDIR ${WORKSPACE_ROOT}
COPY . ${WORKSPACE_ROOT}

# Configurar ROS
ENV DISPLAY=:2

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-ament-package \
    build-essential && \
    rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install ros-humble-ros-gz --fix-missing -y


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

