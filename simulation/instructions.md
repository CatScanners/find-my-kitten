# Custom simulation setup

**Note:** It is assumed that **QGroundControl** is installed. This guide uses version **v4.4.2**.

## Clone the Repository
```bash
git clone --recursive https://github.com/CatScanners/find-my-kitten.git
```

---

## Install ROS2 Gazebo Bridge

Install the ROS2 Gazebo bridge package:

```bash
sudo apt install ros-humble-ros-gzharmonic
```

Before running the simulation, start the image bridge:

```bash
ros2 run ros_gz_image image_bridge /camera
```

This command publishes images from Gazebo as ROS2 topics.

---

## Navigate to the Correct Directory and Run Setup
After cloning the repository, navigate to the `simulation` directory and run the setup script, which initializes submodules and PX4-Autopilot and Micro-XRCE-DDS-Agent directories.

```bash
cd find-my-kitten/ros2_ws/src/simulation/
./setup.sh
```

---

## Setup QGroundControl Communication
### Windows:
1. Open **QGroundControl**.
2. Click the **logo** in the upper-left corner.
3. Go to **Application Settings** → **Comm Links**.
4. Click **Add** to create a new communication link for Gazebo.
5. Provide a name (e.g., `Gazebo Link`).
6. Set **Type** to `UDP`.
7. Set **Port** to `18570`.
8. Click **OK** and **Connect**.

### Linux or WSL2:
In Linux QGroundControl should connect automatically to Gazebo simulator, when simulator is running.

---

## Start Micro-XRCE-DDS Agent
Navigate to the `Micro-XRCE-DDS-Agent/build` directory and start the agent:

```bash
cd Micro-XRCE-DDS-Agent/build
MicroXRCEAgent udp4 -p 8888
```

---

## Run PX4 SITL Simulation
Open a **new terminal** and navigate to `PX4-Autopilot`. Run one of the following commands:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500_gimbal
```

- To launch a world with trees, use:

```bash
make px4_sitl gz_x500_gimbal_baylands
```

A GUI window should appear with the simulated drone. If you want to see other worlds or models, please go to `find-my-kitten/simulation/PX4-Autopilot/Tools/simulation/gz/`. In this directory, you will find folders for models and worlds that can be used in the simulation. To use them, replace `x500_gimbal` with the model you want and `baylands` with the world you want. 

If you want to modify predefined models and worlds, please do so in this repository: [PX4 Gazebo Models](https://github.com/CatScanners/PX4-gazebo-models).

Remember to update submodules after making changes to the `PX4-gazebo-models` repository to apply the latest changes.

---

### Actions startup
Open a new terminal window and navigate to find-my-kitten/ros2_ws directory.
```bash
cd find-my-kitten/ros2_ws
```

Build ros2 directories.
```bash
colcon build
```

Remember to source after build.
```bash
source install/setup.bash
```

1. Start a node receiving offboard messages and then sending those periodically to PX4 simulator. `ros2 run px4_handler offboard_control`. By default just sends current coordinates.
2. Turn on offboard node from QGroundControl. Now the drone is flying with our ROS2 messges, currently just staying in its current position. Again, if you don't have QGC, please refer to the Common problems -section.
3. Star vision package to detect the ball.
`ros2 run vision_package object_detector.py --ros-args -p input_topic_name:="camera" -p output_topic_name:="detections"`
4. a - Start ballfinder node to start searching for the ball:
`ros2 run px4_handler ball_finder.py`, or b - Start predefined motions with two speeds: `ros2 run px4_handler hard_motions.py`.
5. a - With default code, the drone should search an area of 8m x 12m and stop and go a bit down when it sees a **sports ball**, or b - Does a few predefined motions in two different speeds.

---

## **Errors Encountered and Possible Fixes**
Sometimes, PX4-Autopilot might fail to start properly. Below is an example error message you may see:

```
INFO  [gz_bridge] Requested Model Position: 0,0,0,0,0,0
WARN  [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.
ERROR [gz_bridge] Task start failed (-1)
ERROR [init] gz_bridge failed to start and spawn model
ERROR [px4] Startup script returned with return value: 256
```

### **Quick Fix**
If you encounter this error, try stopping the execution of `make px4_sitl gz_x500_gimbal` and **running it again**.

### **Full Reset (If the Issue Persists)**
If restarting doesn't help, try **cleaning the build environment** before rebuilding PX4:

```bash
make clean          # Remove compiled binaries but keep configuration files
make distclean      # Remove all compiled files and reset configuration
make px4_sitl       # Rebuild PX4 SITL
make px4_sitl gz_x500_gimbal  # Start the simulation again
```

If this does not help either, then run this command in terminal
```bash
ps aux | grep gz
```

You will see something like this
```
user     776531  0.0  0.1  38840 25204 pts/0    S+   10:23   0:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run ros_gz_image image_bridge /camera
user     776532 16.1  0.7 1094372 113480 pts/0  Sl+  10:23  28:07 /opt/ros/humble/lib/ros_gz_image/image_bridge /camera
👉 user     817310 91.2  4.3 2070316 686916 pts/7  Rl   13:16   0:13 gz sim --verbose=1 -r -s /home/user/src/find-my-kitten/ros2_ws/src/simulation/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf
user     817443  0.0  0.0   6932  2224 pts/7    S+   13:17   0:00 grep --color=auto gz
```

You should look for processes similar to '817310'. These processes sometimes are left running in the background and causes issues as shown above.
Stop these processes
```bash
kill -9 817310
```
where '817310' should be replaced with the actual PID.

After this try to run the simulation again.