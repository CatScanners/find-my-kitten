# Use the official Ubuntu 22.04 image as the base
FROM ubuntu:22.04

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
   ros-humble-cv-bridge

# Set up ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

#### IMPORTANT UNCOMMENT THIS ONCE EVERYTHING IS READY.
#### RUN git clone https://github.com/CatScanners/find-my-kitten.git
RUN git clone --branch add-instructions --single-branch --recursive https://github.com/CatScanners/find-my-kitten.git
WORKDIR /find-my-kitten/simulation

RUN git submodule update --init --recursive

RUN mkdir -p Micro-XRCE-DDS-Agent/build
RUN mkdir -p ws_offboard_control/src/

RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

WORKDIR /find-my-kitten/simulation/PX4-Autopilot/
RUN make px4_sitl

WORKDIR /find-my-kitten/simulation/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install
RUN ldconfig /usr/local/lib/

WORKDIR /find-my-kitten/simulation/PX4_support/src/px4_msgs
RUN git pull origin main

WORKDIR /find-my-kitten/simulation/PX4_support/src/px4_ros_com
RUN git pull origin main

WORKDIR /find-my-kitten/ros2_ws/src/simulation/PX4_support/
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN echo "source find-my-kitten/simulation/ws_offboard_control/install/local_setup.bash" >> ~/.bashrc

# Install ROS 2 Gazebo bridge (ros_gz) for image transport between Gazebo and ROS 2
RUN sudo apt-get install -y ros-humble-ros-gzharmonic

# Install tmux
RUN apt install -y tmux

# Install QGroundControl
WORKDIR /find-my-kitten
RUN useradd -ms /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN chown -R user:user /find-my-kitten
USER user
RUN pip3 install --user PySide2
RUN sudo usermod -a -G dialout user
RUN sudo apt-get remove modemmanager -y
RUN sudo apt install -y \
	 gstreamer1.0-plugins-bad \
	 gstreamer1.0-libav \
	 gstreamer1.0-gl \
	 libfuse2 \
	 fuse \
	 libxcb-xinerama0 \
	 libxkbcommon-x11-0 \
	 libxcb-cursor-dev \
	 libpulse-dev
RUN sudo wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
RUN sudo chmod +x ./QGroundControl.AppImage

# Bash script for simulation setup (opens and runs all necessary scripts)
COPY startsim.sh /find-my-kitten/startsim.sh
RUN sudo chmod +x /find-my-kitten/startsim.sh

CMD ["/bin/bash"]
