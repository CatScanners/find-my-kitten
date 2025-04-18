# These commands are taken from official PX4 documentation:
# https://docs.px4.io/main/en/ros2/user_guide#install-px4
# https://docs.px4.io/main/en/ros2/offboard_control.html

#!/bin/bash
set -e  # Exit on error

echo "Initializing Git Submodules..."
git submodule update --init --recursive

# -------------------------------
# PX4 Installation
# -------------------------------
echo "Setting up PX4..."
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Build
cd PX4-Autopilot/
make px4_sitl
# Return to simulation directory
cd ..

# -------------------------------
# Micro-XRCE-DDS-Agent Installation
# -------------------------------
echo "Building Micro-XRCE-DDS-Agent..."
cd Micro-XRCE-DDS-Agent

# Create build directory if not exists
mkdir -p build
cd build

# Build and install
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# Return to simulation directory
cd ../../


# -------------------------------
# ROS 2 PX4 Workspace Setup
# -------------------------------
echo "Setting up ROS 2 PX4 workspace..."

# Ensure colcon workspace directory exists
mkdir -p ws_offboard_control/src/
cd ws_offboard_control/src/

# Pull the latest updates for px4_msgs
cd px4_msgs
git pull origin main
cd ..

# Pull the latest updates for px4_ros_com
cd px4_ros_com
git pull origin main
cd ..

# Build the workspace.
cd ../

# Ensure that ROS 2 Humble is installed.
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS 2 Humble is not installed. Please install it before running this script."
    exit 1
fi

source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

echo "Setup completed successfully!"