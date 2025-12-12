FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]

# Avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Basic dependencies
RUN apt-get update && apt-get -qq install -y --no-install-recommends \
    locales \
    wget \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    fuse \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    libpulse-dev \
    python3-pip \
    ros-humble-ros-gz \
    tmux && \
    rm -rf /var/lib/apt/lists/

RUN wget -q \
    https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage \
    -O /usr/local/bin/QGroundControl && \
    chmod +x /usr/local/bin/QGroundControl

# Make sure locale is UTF-8
RUN locale-gen en_US en_US.UTF-8 && \
   update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
   export LANG=en_US.UTF-8

# Setup tzdata
RUN echo "Europe/Helsinki" > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata

RUN useradd -ms /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG dialout user

USER user
ENV USER=user

COPY --chown=user:user simulation/PX4-Autopilot/ /find-my-kitten/simulation/PX4-Autopilot/
WORKDIR /find-my-kitten/simulation/

RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    sudo rm -rf /var/lib/apt/lists/

# PX4-Autopilot needs these files to build
COPY --chown=user:user \
    .git/modules/simulation/PX4-Autopilot/ \
    /find-my-kitten/.git/modules/simulation/PX4-Autopilot/
WORKDIR /find-my-kitten/simulation/PX4-Autopilot/

# After building first time, only the HEAD and index files are required,
# remove everything else to save space
RUN make -j$(nproc) px4_sitl && \
    find /find-my-kitten/.git/modules/simulation/PX4-Autopilot/* \
        -maxdepth 0 ! -name "HEAD" ! -name "index" -exec rm -rf {} +

ENV GZ_SIM_RESOURCE_PATH="/find-my-kitten/simulation/PX4-Autopilot/Tools/simulation/gz/models:/find-my-kitten/simulation/PX4-Autopilot/Tools/simulation/gz/worlds"

COPY --chown=user:user simulation/Micro-XRCE-DDS-Agent/ /find-my-kitten/simulation/Micro-XRCE-DDS-Agent/
WORKDIR /find-my-kitten/simulation/Micro-XRCE-DDS-Agent
RUN cmake . -B build && \
    make -C build -j $(nproc) && \
    sudo make -C build install && \
    rm -rf /find-my-kitten/simulation/Micro-XRCE-DDS-Agent/ && \
    sudo ldconfig /usr/local/lib/

# COPY --chown=user:user simulation/PX4_support/ /find-my-kitten/simulation/PX4_support/
# COPY --chown=user:user ros2_ws/src/px4_msgs/ /find-my-kitten/ros2_ws/src/px4_msgs/
# WORKDIR /find-my-kitten/simulation/PX4_support/
# RUN source /opt/ros/humble/setup.bash && colcon build

# RUN pip3 install --user PySide2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Bash script for simulation setup (opens and runs all necessary scripts)
COPY --chown=user:user ./simulation/startsim.sh /find-my-kitten/simulation/startsim.sh
RUN chmod +x /find-my-kitten/simulation/startsim.sh

WORKDIR /find-my-kitten/simulation
