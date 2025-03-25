FROM osrf/ros:humble-desktop

# Avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update apt-get and install basic dependencies
RUN  apt-get update && apt-get upgrade -y && apt-get install -y \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    python3-pip \
    python3-venv \
    build-essential \
    locales \
    tzdata

# Make sure locale is UTF-8
RUN locale-gen en_US en_US.UTF-8 && \
   update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
   export LANG=en_US.UTF-8

# Clone and setup PX4-Autopilot fork
RUN git clone --recursive https://github.com/CatScanners/PX4-Autopilot.git -b px4-v1.15.4

# Setup PX4-simulation
# NOTE! change setup/ubuntu.sh
RUN apt-get update && apt-get install -y wget
RUN cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh
    
RUN cd PX4-Autopilot && make px4_sitl

# Clone Micro-XRCE-DDS-Agent repo  
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

# Setup Micro-XRCE-DDS-Agent
RUN cd Micro-XRCE-DDS-Agent \
  mkdir build \
  cd build \
  cmake .. \
  make -j$(nproc) \
  sudo make install \
  sudo ldconfig /usr/local/lib/

# Install the ROS2 Gazebo bridge package
# RUN sudo apt install ros-humble-ros-gzharmonic

# ADD QGround Install 