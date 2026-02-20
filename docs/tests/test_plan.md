# Tests

## System requirements
- Ubuntu 24, aka Ubuntu Jazzy
- 150 GB of storage (excl. the OS)
- Nvidia GPU (only required for simulator tests)
- docker (installation instructions can be found from: https://docs.docker.com/engine/install/ubuntu/)


## Initial Setup 
**NOTE: Make sure to have installed git-lfs, after which you should cloned the repository. <br/>
If you have already cloned the repository, run the following commands:**
```
git lfs fetch
git lfs pull
```

Before beginning the tests, you'll need to download and install the following dependencies and drivers. <br/>

You can install Ubuntu drivers by running: `sudo ubuntu-drivers install` <br/>

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


## Plan 1. Running the simulator inside docker container
Before running the simulation inside a container, you'll first have to build ros2 by following these steps: <br/> 

```
cd into /ros2_ws
sudo ./start_isaac_dev.sh
```

If after the last command you get and error and the message: "have you initialized submodules?", circle back to the git-lfs section of [Initial setup](#initial-setup)

If the script runs without errors, then you will notice that it starts the docker container. This step can take several tens of minutes, so you'll have to wait a while. <br/>


After you have successfully started the container, cd into /ros2_ws in the container, and build the package by running: <br/>
```
colcon build
```

After building, return to the parent directory and cd into /simulation , where you can start the simulation by running: <br/>
```
./start_isaac_sim.sh
```