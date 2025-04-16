# Docker simulation setup guide

This guide walks you through setting up the **ROS 2 + Gazebo + PX4 simulation** environment inside Docker using the `find-my-kitten` repository.

---

## Prerequisites

- **Docker must be preinstalled**
- Make sure that there is an access to Docker (or use `sudo`)
- Clone the repository using:

```bash
git clone --recursive git@github.com:CatScanners/find-my-kitten.git
```

---

## Build and run the Docker container

1. **Navigate to the simulation directory**:

   ```bash
   cd find-my-kitten/ros2_ws/src/simulation
   ```

2. **Build the Docker container**:

   ```bash
   sudo docker build -t kitten-sim .
   ```

3. **Run the container**:

   ```bash
   sudo docker run -it \
     --privileged \
     --device /dev/fuse \
     --cap-add SYS_ADMIN \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     kitten-sim
   ```

---

## Use `tmux` for Multiple Terminal Panes

4. **Start `tmux` inside the container**:

   ```bash
   tmux
   ```

5. **Useful `tmux` shortcuts**:

| Action                        | Shortcut              |
|------------------------------|-----------------------|
| Split pane vertically        | `Ctrl + b`, then `%`  |
| Split pane horizontally      | `Ctrl + b`, then `"`  |
| Close current pane           | `Ctrl + b`, then `x`  |
| Switch between panes         | `Ctrl + b`, then arrow keys |

> **After opening each new pane, run:**

```bash
source /opt/ros/humble/setup.bash
```

---

## Start the simulation components

6. **In the first pane**, launch QGroundControl:

```bash
./QGroundControl.AppImage
```

7. **In another pane**, start the ROS-Gazebo image bridge:

```bash
ros2 run ros_gz_image image_bridge /camera
```

8. **In another pane**, run the DDS Agent:

```bash
cd find-my-kitten/ros2_ws/src/simulation/Micro-XRCE-DDS-Agent/build/
MicroXRCEAgent udp4 -p 8888
```

9. **In another pane**, launch the PX4 simulation:

```bash
cd find-my-kitten/ros2_ws/src/simulation/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_lawn
```

**QGroundControl** should automatically connect to the PX4 simulation.
Just open the application window if it isn't already visible.

