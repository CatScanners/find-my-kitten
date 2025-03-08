# Use the official Ubuntu 22.04 image as the base
FROM ubuntu:22.04

# Avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Build argument for including DepthAI ROS package. Defaults to false
ARG DEPTHAI=false

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

# Setup tzdata
RUN echo "Europe/Helsinki" > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata

# Install ROS 2 Humble
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install apt ros & depedencies
RUN sudo apt-get update && sudo apt-get install -y \
   ros-humble-ros-base \
   ros-dev-tools \
   ros-humble-image-transport \
   python3-opencv \
   ros-humble-ament-cmake \
   ros-humble-ament-cmake-python \
   libopencv-dev \
   ros-humble-cv-bridge \
   ros-humble-depthai-ros

# Install depthai ros binaries if the flag is set as true
RUN if [ "$DEPTHAI" = "true" ]; then apt install -y ros-humble-depthai-ros; fi 

 # Set up ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Copy vision_package
COPY . /vision_package/
WORKDIR /vision_package

# Install pip dependencies listed in requirements.txt
RUN pip3 install -r requirements.txt

# Give python scripts execution rights
RUN chmod +x ./vision_package/vision_package/*

# Use bash
SHELL ["/bin/bash", "-c"]

# Setup Rosdep and install dependencies
RUN rosdep init && \
   rosdep update && \
   rosdep install --from-paths . --ignore-src -r -y


# Build the package & source
RUN  source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash


# Entry point 
CMD ["/bin/bash"]

