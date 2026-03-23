# Tests

## System requirements
- Ubuntu 24, aka Ubuntu Jazzy
- 150 GB of storage (excl. the OS)
- Nvidia GPU (required for simulator tests)
- docker (installation instructions can be found at: https://docs.docker.com/engine/install/ubuntu/)


## Initial Setup 
**NOTE: Make sure to have installed git-lfs, after which you should cloned the repository. <br/>
If you have already cloned the repository, run the following commands:**
```
git lfs fetch
git lfs pull
```

Before beginning the tests, you'll need to download and install the dependencies and drivers defined below: <br/>

Install necessary Ubuntu drivers by running: `sudo ubuntu-drivers install` <br/>

Make sure you have **python3 version 3.12.4** or newer installed. <br/>

For docker containers you also have to install Nvidia Container Toolkit by running the following commands: <br/> 
(commands taken from Nvidia docs at: 
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) <br/>


```
sudo apt-get update && sudo apt-get install -y --no-install-recommends \ 
   ca-certificates \ 
   curl \ 
   gnupg2
```

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```
sudo apt-get update
```

```
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.2-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

<br/> 


**After installing the drivers, you have to install the remaining apps and projects by following the installation guide given in this repository at the subpath:**
```
/find-my-kitten/docs/IsaacSim/Installing.md
```

or by reading it through this link: https://github.com/CatScanners/find-my-kitten/blob/main/docs/IsaacSim/Installing.md  <br/> 




## Plan 2. Running the simulator inside docker container
### Setup
Before running the simulation inside a container, you'll first have to start the container and build ros2 by following the following steps: <br/> 

```
./start_isaac_dev.sh -sc
```

NOTE: If after the last command you get an error and the message: "have you initialized submodules?", circle back to the git-lfs section of [Initial setup](#initial-setup)

If the script runs without errors, then you will notice that it starts the docker container. This step will take around 30 minutes, so you'll have to wait a while. <br/>


After you have successfully started the container, cd into /ros2_ws in the container, and build the package by running: <br/>
```
colcon build
```

After building, return to the parent directory and run the following list of commands (You can copy paste all of them at once into the terminal and press enter):
```
cd /home/admin/isaacsim/
./post_install.sh
./isaac-sim.selector.sh
source ~/.bashrc
cd /workspaces/isaac_ros-dev
sudo apt update
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
uv sync
uv add smmap gitpython numpy scipy
source .venv/bin/activate
sudo chown -R admin:admin /home/admin/PegasusSimulator
export ISAACSIM_PYTHON=/home/admin/isaacsim/python.sh
export ISAACSIM_PATH=/home/admin/isaacsim
$ISAACSIM_PYTHON -m pip install --editable /home/admin/PegasusSimulator/extensions/pegasus.simulator
cd ros2_ws
source install/setup.bash
source /opt/ros/humble/setup.bash
```

When runnin these commands, you should get a small GUI pop-up that is the setup for the simulator UI.<br/>
In this window, Just press the large green button at the bottom, wait for a larger UI to open, and then you can close both GUI's and go to the next step. <br/>

<br/>

### Runnig the simulator
After the last series of commands, you should be at the path: `/workspaces/isaac_ros-dev/ros2_ws` <br/>
In this folder, you can start the simulation by running the following command: <br/>
`ros2 launch kitten_sim kitten_sim.launch.py`