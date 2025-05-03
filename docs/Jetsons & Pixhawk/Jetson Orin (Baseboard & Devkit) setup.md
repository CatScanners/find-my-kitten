---
parent: Jetsons & Pixhawk setup
title: Baseboard setup - deviations from tutorial
---

# Baseboard setup - deviations from tutorial
Applicable to:
- Baseboard

The following tutorial was used for the HolyBro Jetson baseboard and ethernet link between the Jetson and the baseboard: <https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html>.

The goals were achieved, albeit with slightly different configurations. Unlike what the guide implies, the pixhawk required setting the static ip mentioned. This was done in the MAVLINK console with the following commands:
```
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.41.10.2 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg
echo ROUTER=10.41.10.254 >>/fs/microsd/net.cfg
echo DNS=10.41.10.254 >>/fs/microsd/net.cfg
netman update
```

The netplan configuration file `/etc/netplan/01-netcfg.yaml` did not work for our purposes, which were to both be able to communicate with internet via the RJ45 port, and to the pixhawk via the ethernet bridge.
Communications with the internet did not work originally. This was fixed by adding an additional route and static address, from which the config file finally got the form:
```
network:
  version: 2
  renderer: networkd
  ethernets:
    enP8p1s0:
      dhcp4: yes
      addresses: [10.41.10.1/24, ADDR.1/25]
      routes:
        - to: 0.0.0.0/0
          via: 10.41.10.254
          metric: 100
        - to: 0.0.0.0/0
          via: ADDR.254
          metric: 200
      nameservers:
        addresses: [10.41.10.254, 8.8.8.8, 8.8.4.4]
```
Where ADDR denotes the first parts of the IPV4 address under enP8p1s0 when displaying `ip addr show`, i.e. `222.222.222.55` -> `ADDR = 222.222.222`. Note the subdomain `/25`, which might be different in your case.

Notice that the PX4 parameters may have to be re-configured when updating/reflashing.

Instead of the given `pip install --user -U empy==3.3.4 pyros-genmsg setuptools`, the following were used:
```
sudo apt-get install python3-genmsg
sudo apt-get install python3-setuptools
```

# Jetson Orin installation
Applicable to:
- Devkit

This section applies to the Jetson Orin Nano Super Devkit. Based on [Nvidia's getting started](<https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>) guide.

Needed:
- Jetson Orin Nano devkit with power supply
- microSD card (+ NVMe SSD for devkit)
- Keyboard
- Display (DisplayPort cable)
- 2nd computer with SD card reader

## Firmware update & OS installation
Based on [Jetson AI Lab](<https://www.jetson-ai-lab.com/initial_setup_jon.html>) documentation

Connect the monitor and keyboard to the devkit. Plug in the power supply to turn it on and start repeatedly pressing the Esc key on the keyboard to enter the UEFI/BIOS setup menu. Check the third line from the top for the Jetson UEFI firmware.

If Firmware < 36.0:
- Download the [Jetpack 5.1.3](<http://developer.nvidia.com/downloads/embedded/l4t/r35_release_v5.0/jp513-orin-nano-sd-card-image.zip>). Otherwise download [the newest](<https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.0/jp61-orin-nano-sd-card-image.zip>) SDK.

After you have the image, use your favored live USB utility such as Rufus or [Balena Etcher](<https://etcher.balena.io/>) to flash it onto the microSD card with the other computer. Next, insert the microSD card into the Jetson and power it back on. Complete the software setup.

If Firmware < 36.0:
- Wait for `nvidia-l4t-bootloader` to prompt a restart for an update & restart and wait for the update to finish. Then open terminal and execute `sudo nvbootctrl dump-slots-info` and `sudo apt-get install nvidia-l4t-jetson-orin-nano-qspi-updater` and wait for a new update prompt & proceed to restart. After it finishes, the system will get stuck in the boot process and you should unplug the power, install the newest Jetpack SDK and flash it onto the SD card.

After everything is done, do a simple `sudo apt-get update` and `sudo apt-get upgrade` to otherwise update to the newest versions. You might be prompted for another `nvidia-l4t-bootloader` update requiring restart.

# SSD & Docker
Applicable to:
- Baseboard
- Devkit

This section applies to both the devkit and baseboard, but differ in that the baseboard already has the OS install on the SSD while the devkit requires the SSD to be specified for Docker. It is a requirement for setting up Isaac ROS, but almost everything in this project has also been tested in a host environment as well.

For setting up docker & an NVMe SSD, follow [this](<https://www.jetson-ai-lab.com/tips_ssd-docker.html>) guide from Jetson AI Lab.

# Installing ROS2 Humble
Applicable to:
- Baseboard
- Devkit

This section is about installation on the host environment though ideally everything should run in a container (such as with Isaac ROS) for reproducibility:
- <https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_storage.html>
- <https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html>

Based on [ROS2 Installation docs](<https://docs.ros.org/en/humble/Installation.html>)

Verify utf-8 locale (need to change if not):
```
locale
```

```
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
```

Installing the full, development version
```
sudo apt install ros-humble-desktop
```

For deployment, we might want to consider the barebones, without GUI tools
```
sudo apt install ros-humble-ros-base
```

Dev tools
```
sudo apt install ros-dev-tools
```

Add sourcing to shell startup script
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Isaac ROS installation
Applicable to:
- Baseboard
- Devkit

Set the Jetson to maximum performance (MAXN power and locked boost) which will make building a lot faster.

```
sudo nvpmodel -m 0
sudo /usr/bin/jetson_clocks
```

Based on [Nvidia Isaac ROS Getting Started](<https://nvidia-isaac-ros.github.io/getting_started/index.html>)

Following the Docker setup, SSD is initialized to `/ssd/` on the devkit (`/` on baseboard) and set as the default location for docker. We can now proceed with ROS2 Docker setup.

Based on [Developer Environment Setup](<https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html>), adapted to SSD mounting location. Also following [Isaac Apt Repository](<https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html>), 

Install Git LFS

```
sudo apt-get install git-lfs
git lfs install --skip-repo
```

Create a ROS2 workspace and create an environment variable for it, required by the Isaac ROS dev scripts.

```
mkdir -p /ssd/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=/ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

Go to the WS and clone the Isaac ROS Common repository

```
cd $ISAAC_ROS_WS/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```


Deploy [Isaac ROS Dev](<https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment>) Docker (sets up all basic dependencies including ROS2 Humble)

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

Took more than 5000 seconds to run on 1 Gb/s WiFi, perhaps Ethernet is better. (Note that I ran `cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh` without the `-d` flag, not sure if it ended up installing on the SD card but storage usage implies no)

`cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh` can also be used to enter the Docker environment after it is installed. [This](<https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment>) provides documentation for the scripts, but the gist is that it creates a base image with [aarch64](<https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.base>) and [ros2_humble](<https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.ros2_humble>) layers built on `nvidia/cuda:12.6.1-devel-ubuntu22.04` and runs the container with a lot of arguments to pass hardware acceleration and also bind mount the directory `$ISAAC_ROS_WS` meaning all packages built inside should persist.

The Docker image can be extended by creating more Dockerfiles and specifying their order in the `run_dev.sh` script via the arg `-i <layer1_name>.<layer2_name>.<layer3_name>`, for example with our custom Dockerfiles on [our fork](<https://github.com/CatScanners/isaac_ros_common/tree/release-3.2/docker>)

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh -i "ros2_humble.catscanners_ws.zed.yolo"
```


# SSH server
Applicable to:
- Baseboard
- Devkit

Generate a host key first
```
ssh-keygen -t rsa -b 4096
```

Create a new config file to block password-based authentication

```
sudo vim /etc/ssh/sshd_config.d/disable_root_login.conf
```

Press insert to use edit mode and write the following:

```
ChallengeResponseAuthentication no
PasswordAuthentication no
UsePam no
PermitRootLogin
```

Press esc to exit edit mode, then ":wq" to save and quit

Add users to authorized keys

```
sudo apt-get-install ssh-import-id
ssh-import-id gh:<username>
```

Enable & reload SSH

```
sudo systemctl enable ssh
sudo systemctl reload ssh
sudo systemctl reload sshd
```

Verify the active config

```
sudo sshd -T | grep -E "passwordauthentication|pubkeyauthentication|usepam|permitrootlogin"
```