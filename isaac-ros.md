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