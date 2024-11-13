
# Setting up the hardware

Based on [Nvidia's getting started](<https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>) guide.

Needed:
- Jetson Orin Nano devkit with power supply
- microSD card (+ NVMe SSD for devkit)
- Keyboard
- Display (DisplayPort cable)
- 2nd computer with SD card reader

# Firmware update & OS installation
Based on [Jetson AI Lab](<https://www.jetson-ai-lab.com/initial_setup_jon.html>) documentation

Connect the monitor and keyboard to the devkit. Plug in the power supply to turn it on and start repeatedly pressing the Esc key on the keyboard to enter the UEFI/BIOS setup menu. Check the third line from the top for the Jetson UEFI firmware.

If Firmware < 36.0:
- Download the [Jetpack 5.1.3](<http://developer.nvidia.com/downloads/embedded/l4t/r35_release_v5.0/jp513-orin-nano-sd-card-image.zip>). Otherwise download [the newest](<https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.0/jp61-orin-nano-sd-card-image.zip>) SDK.

After you have the image, use your favored live USB utility such as Rufus or [Balena Etcher](<https://etcher.balena.io/>) to flash it onto the microSD card with the other computer. Next, insert the microSD card into the Jetson and power it back on. Complete the software setup.

If Firmware < 36.0:
- Wait for `nvidia-l4t-bootloader` to prompt a restart for an update & restart and wait for the update to finish. Then open terminal and execute `sudo nvbootctrl dump-slots-info` and `sudo apt-get install nvidia-l4t-jetson-orin-nano-qspi-updater` and wait for a new update prompt & proceed to restart. After it finishes, the system will get stuck in the boot process and you should unplug the power, install the newest Jetpack SDK and flash it onto the SD card.

After everything is done, do a simple `sudo apt-get update` and `sudo apt-get upgrade` to otherwise update to the newest versions. You might be prompted for another `nvidia-l4t-bootloader` update requiring restart.

# SSD & docker
For setting up docker & an NVMe SSD, look at [this](<https://www.jetson-ai-lab.com/tips_ssd-docker.html>).

# WIP
## Installing ROS2
Based on [ROS2 Installation docs](<https://docs.ros.org/en/iron/Installation.html>)

Verify utf-8 locale:
```
locale
```

```
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update
sudo apt upgrade
```

Installing the full, development version
```
sudo apt install ros-iron-desktop
```

For deployment, we might want to consider the barebones, without GUI tools
```
sudo apt install ros-iron-ros-base
```

Set up the environment
```
source /opt/ros/iron/setup.bash
```

## isaac_ros

Based on [Nvidia Isaac ROS Getting Started](<https://nvidia-isaac-ros.github.io/getting_started/index.html>)

It appears isaac_ros is already set up out of the box?

https://docs.nvidia.com/jetson/jetpack/install-setup/index.html


Set the Jetson to maximum performance
```
sudo /usr/bin/jetson_clocks
```
