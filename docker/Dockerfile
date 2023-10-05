FROM osrf/ros:humble-desktop as base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
 git wget libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Create a workspace and run the PX4 setup to install all requirements for simming
RUN mkdir -p /DroneWorkpace/sim
WORKDIR /DroneWorkpace/sim
COPY ./docker/px4_setup.sh .
COPY ./docker/requirements.txt .
COPY ./sim/PX4-Autopilot/ ./
RUN bash ./px4_setup.sh

# Set up the entrypoint
WORKDIR /DroneWorkspace
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#################
# Overlay Image #
#################
FROM base AS overlay

# Create an overlay Colcon workspace
WORKDIR /DroneWorkpace
RUN mkdir src
WORKDIR /src
COPY ./src/gtsam ./
COPY ./src/px4_msgs ./

# Set up the entrypoint
WORKDIR /DroneWorkpace
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]