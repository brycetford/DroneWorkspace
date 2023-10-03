FROM osrf/ros:humble-desktop as base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
 git wget libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Create a workspace and copy in the PX4 submodule and run the setup
RUN mkdir -p /DroneWorkpace/submodules
WORKDIR /DroneWorkpace/submodules
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git
RUN ./PX4-Autopilot/Tools/setup/ubuntu.sh
