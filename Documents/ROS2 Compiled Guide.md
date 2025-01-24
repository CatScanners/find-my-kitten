# ROS2 Compiled Guide

## Table of Contents
- [Introduction](#introduction)
- [ROS2](#ros2)
  - [Abstract](#abstract)
  - [Install](#install)
  - [How-To-Basics](#how-to-basics)
- [Isaac ROS](#isaac-ros)
  - [Abstract](#isaac-ros-abstract)
  - [Install](#isaac-ros-install)
  - [How-To-Basics](#isaac-ros-how-to-basics)
- [Links](#links)

## Introduction
The aim of this document is to have the core ROS2 knowledge in a centralized place. \
**This should not include anything specific to the find-my-kitten project.**  \
Just a generic cheat sheet for developing ROS2.

# ROS2

## Abstract
ROS 2 data distribution service consists of communication pipelines and nodes. 
Nodes have three ways to communicate:
  #### 1. Publisher/Subscriber
  - In this context pipelines are called *topics* in which *publisher* node publishes information, referred as *message*
  - *Subscriber* nodes of a topic receive published messages.
  #### 2. Services
  - Node sends *request* to other node (e.g., turn camera 45 degrees).
  - The receiving node follows the request with a *respond* to the requesting node (e.g., with camera image).
  #### 3. Actions
  - *Client* node sends *goal* to an *action server* node (e.g., move drone to this location)
  - Action server processes the goal and then sends progress updates, referred as *feedback*
    to the client node (e.g., how far the drone is from the location)
  - When the goal is reached the action server sends a *result* to the client (e.g., picture of the location)


## Install
Below one can find two different ROS2 distro install guides.
- [Humble Ubuntu Install Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Rolling Ubuntu Install Guide](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html) 

### apt:
- Enable Ubuntu universe repository\
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add ROS2 GPG keys
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings ros-archive-keyring.gpg
```
Add ros2 repo to apt sources
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Update and upgrade apt afterwards.
```
sudo apt update && apt upgrade
```
Now install ROS-Base nad ros devtools. Choose distro by specifying ros-**DISTRO**-ros-base. In example humble.  
```
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```
Now source the enviroment (notice distro)
```
source /opt/ros/humble/setup.bash
```
Sourcing needs to be done in each shell. Add the above code to your .bashrc to automatically source ros2. 
\
Functionality can be confirmed with simply calling ros2. 
```
ros2 --help
```
### Docker:
Ready made docker containers are also available, with most dependencies already included. Please keep in mind this is not a docker tutorial, just a "hey this thing exists".  \
I've personaly tested **ros:humble** and **osrf/ros:rolling-desktop** and both were old but functional.
<br><br>
Running a docker container is as simple as:

```
sudo docker run -it --priviliged --name ros2 ros:humble
```
Afterwards you are in a shell where you can conduct the testing and development. \
These containers are old so do update you system with:
```
sudo apt update && upgrade
```
Afterwards you can confirm that ros2 is indeed preinstalled:
```
ros2 --help
```
## How-To-Basics
### 0. Package structure

Simplest possible cpp package has following file structure:
```
  my_package/
      CMakeLists.txt
      include/my_package/
      package.xml
      src/
```
And python package:
```
  my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/

```
Explanations for these required package contents can be found in the [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), but shortly: 
- **package.xml** defines the package details like name, version and license. Also dependencies are defined here for [rosped](http://wiki.ros.org/rosdep).
- **include/my_package** or **resource/my_package** allows as an example file **include/my_package/variables.hpp** to be included in the project src code.
```
#include <vision_package/variables.hpp>
```
- **CMakeList.txt** /  **setup.py** are needed for building the package and specify dependencies on a node level. 
- **setup.cfg** is for enabling python to use executables. So running c++ for an example.  
### 1. Create a workspace

Make workspace directory (name can be whatever) and make `src` subdirectory in it:
```
mkdir -p ~/ros2_ws/src
```

All packages in your workspace should be put into the `src` directory.



### 2. Create a package
First `cd` into `src` folder
```
cd ~/ros2_ws/src
```
Then use following command to make cpp package:
```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```
or command to make python package:
```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
Now there is a new folder in `src` directory called `my_package`.

### 3. Write your code and fill package information. 

Write your code and then specify the dependencies that are used in the package.xml and other files. \
Some ROS specific dependencies can be installed with [rosped](http://wiki.ros.org/rosdep). \
For first use run commands:
```
sudo rosdep init
rosdep update
```
And after that dependecies can be installed when in the project directory with: 
```
rosdep install --from-paths . --ignore-src -r -y
```
"This command magically installs all the packages that the packages in your catkin workspace depend upon but are missing on your computer." But some ROS2 packages might need to be installed separately like image_transport, which can be found from apt. 
``` 
sudo apt install ros-humble-image-transport
```
### 4. Build packages
`cd` to the root folder
```
cd ~/ros2_ws
```
To build all packages in the workspace use:  
```
colcon build --symlink-install 
```
To build only `my_package` use:
```
colcon build --packages-select my_package
```

### 5. Source the setup file
First open a new terminal. Sourcing should always done in new terminal after building to avoid complex issues.

Then `cd` into `ros2_ws` and run the following command:
```
source install/local_setup.bash
```
### 6. Use the package
To run `my_node`, enter the command:
```
ros2 run my_package my_node
```

# Isaac ROS

## Isaac ROS Abstract
Nvidias Isaac ROS is a ros2 "distribution" with packages and dependencies preinstalled and optimized for Nvidia Jetson platforms. It is not a distro per say as and can be downloaded based on ROS2 Humble or Foxy distros. It is fully compatible with normal ROS2 but just branded with Nvidia.

## Isaac ROS Install
following [setup](<./jetson-setup.md>), SSD is initialized to `/ssd/` and set as the default location for docker. We can now proceed with ROS2 Docker setup.

Based on [Developer Environment Setup](<https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html>), adapted to SSD mounting location. Also following [Isaac Apt Repository](<https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html>), 

Install Git LFS
```
sudo apt-get install git-lfs
git lfs install --skip-repo
```

Create a ROS2 workspace
```
mkdir -p /ssd/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=/ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

Go to the WS and pull the repo

```
cd $ISAAC_ROS_WS/src
git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```

Set performance and power to max
```
sudo /usr/bin/jetson_clocks
sudo /usr/sbin/nvpmodel -m 0
```


Deploy [Isaac ROS Dev](<https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment>) Docker (sets up all basic dependencies including ROS2 Humble)

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

Took more than 5000 seconds to run on 1 Gb/s WiFi, perhaps Ethernet is better. (Note that I ran `cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh` without the `-d` flag, not sure if it ended up installing on the SD card but storage usage implies no)

`cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh` can also be used to enter the Docker environment after it is installed.

## Isaac ROS How-To-Basics
Basic usage of Isaac ROS.

# Links
- [rosdep Wiki](https://wiki.ros.org/rosdep)

