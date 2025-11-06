FROM ghcr.io/sloretz/ros:humble-desktop-full AS base

ARG WORKSPACE_ROOT="/ros2-bridge"
ARG PACKAGE_NAME="ros2_bridge"

# Verificar a arquitetura primeiro
RUN echo "Building on architecture: $(uname -m)"

# Instalação de utilitários
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat build-essential cmake \
    ros-humble-foxglove-bridge ros-humble-depthai-ros

# SOLUÇÃO: Remover a instalação problemática e usar pacotes disponíveis
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Tentar instalar gazebo-ros2-control separadamente ou usar alternativa
RUN apt-get update && apt-get install -y ros-humble-gazebo-ros || \
    echo "gazebo-ros2-control não disponível, continuando sem ele..."



WORKDIR ${WORKSPACE_ROOT}

# Copy everything into the workspace (except what's in .dockerignore)
COPY . ${WORKSPACE_ROOT}


# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2

# Rosdep get packages
RUN rosdep update && rosdep install --from-paths . --ignore-src -r -y

# Instalar controlador diff-drive (já deve estar incluído no desktop-full)
RUN apt-get install -y ros-${ROS_DISTRO}-diff-drive-controller

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

RUN colcon build 