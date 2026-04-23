# Peer Testing Plan

The peer testing plan consists of two parts: testing the completeness of the documentation within the repository, and setting up and running the IsaacSim simulator.

## Repository Documentation

The goal of this section is to assess whether the repository’s documentation is complete, clear, and accessible for new developers. This includes checking for the presence of essential documentation, its organization, and its ability to guide a new contributor through setup, development, and troubleshooting.

The entry point to documentation is the docs/ folder.

### Step 1 - Broad Check

- Does the README explain what this project is and how to get started or is it just a wall of text?
- Are there clear step-by-step guides for setting things up?
- Are there rules for contributing and submitting code, or is it a free-for-all?

Basically, skim the repo and note down any obvious gaps.

### Step 2 - Dive Deeper

- Are steps actually doable, or do they assume you already know everything?
- Are there code snippets or screenshots to show how things work?
- Are dependencies listed explicitly? Are you told what you need before you start?
- If something goes wrong, does the doc tell you how to fix it?

Ideally each member of the peer testing team picks one thing (like the installation guide) and tries to follow it. If you get stuck, note where and why.

### Step 3 - Organization

- Is the structure of the documentation clear or are files scattered everywhere?
- Are the headings or table of contents clear?
- Can you search for keywords (like “install” or “setup”) and find answers quickly?
- Are links to other docs or tools actually clickable and working?

Note down any problems.

### Step 4 - Summarize

Report all issues in one document and share it with us.

## Simulator Setup

The second part of the peer testing session consists of installing the simulator and runnign a simple scenario. It is limited to users who can fulfill the requirements.

### System requirements

- Ubuntu 24, aka Ubuntu Jazzy
- 150 GB of storage (excl. the OS)
- Nvidia GPU (required for simulator tests)
- docker (installation instructions can be found at: https://docs.docker.com/engine/install/ubuntu/)

For Windows users: this process was tried using WSL with limited success. A full VM might lead to a more promising experience.

### Initial Setup

**NOTE: Make sure to have installed git-lfs, after which you should cloned the repository. <br/>
If you have already cloned the repository, run the following commands:**

```bash
git lfs fetch
git lfs pull
```

Before beginning the tests, you'll need to download and install the dependencies and drivers defined below: <br/>

Install necessary Ubuntu drivers by running: `sudo ubuntu-drivers install` <br/>

Make sure you have **python3 version 3.12.4** or newer installed. <br/>

For docker containers you also have to install Nvidia Container Toolkit by running the following commands: <br/>
(commands taken from Nvidia docs at:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) <br/>

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2
```

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```bash
sudo apt-get update
```

```bash
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.2-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

<br/>

**After installing the drivers, you have to install the remaining apps and projects by following the installation guide given in this repository at the subpath:**

```bash
/find-my-kitten/docs/IsaacSim/Installing.md
```

or by reading it through this link: https://github.com/CatScanners/find-my-kitten/blob/main/docs/IsaacSim/Installing.md <br/>

### Plan 2. Running the simulator inside docker container

#### Setup

Before running the simulation inside a container, you'll first have to start the container and build ros2 by following the following steps: <br/>

```bash
./start_isaac_dev.sh -sc
```

NOTE: If after the last command you get an error and the message: "have you initialized submodules?", circle back to the git-lfs section of [Initial setup](#initial-setup)

If the script runs without errors, then you will notice that it starts the docker container. This step will take around 30 minutes, so you'll have to wait a while. <br/>

After you have successfully started the container, cd into /ros2_ws in the container, and build the package by running: <br/>

```bash
colcon build
```

After building, return to the parent directory and run the following list of commands (You can copy paste all of them at once into the terminal and press enter):

```bash
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

#### Running the simulator

After the last series of commands, you should be at the path: `/workspaces/isaac_ros-dev/ros2_ws` <br/>
In this folder, you can start the simulation by running the following command: <br/>
`ros2 launch kitten_sim kitten_sim.launch.py`
