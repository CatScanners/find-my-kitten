---
parent: Simulation & Flight Analysis
title: Custom simulation setup
---


# Custom simulation setup

**Note:** It is assumed that **QGroundControl** is installed. This guide uses version **v4.4.2**.

## Clone the Repository
```bash
git clone --recursive https://github.com/CatScanners/find-my-kitten.git
```

## Navigate to the Correct Directory and Run Setup
After cloning the repository, navigate to the `simulation` directory and run the setup script:

```bash
cd find-my-kitten/ros2_ws/src/simulation/
./setup.sh
```

---

## Setup QGroundControl Communication
1. Open **QGroundControl**.
2. Click the **logo** in the upper-left corner.
3. Go to **Application Settings** → **Comm Links**.
4. Click **Add** to create a new communication link for Gazebo.
5. Provide a name (e.g., `Gazebo Link`).
6. Set **Type** to `UDP`.
7. Set **Port** to `18570`.
8. Click **OK** and **Connect**.

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
make px4_sitl gz_x500
```

- To launch a world with trees, use:

```bash
make px4_sitl gz_x500_baylands
```

A GUI window should appear with the simulated drone.

---

## Run Offboard Control Node
Without closing previous terminals, open a **new terminal** and navigate to the `ws_offboard_control` directory:

```bash
cd ws_offboard_control
ros2 launch px4_ros_com offboard_control_launch.yaml
```

This will start sending trajectory commands to the simulated drone.

---

## Making Changes to `px4_ros_com`
If you modify any code in `px4_ros_com`, follow these steps before running it again:

1. Open a terminal and navigate to `ws_offboard_control`:
   ```bash
   cd ws_offboard_control
   ```
2. Build the package:
   ```bash
   colcon build
   ```
3. Source the updated environment:
   ```bash
   source install/local_setup.bash
   ```

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
If you encounter this error, try stopping the execution of `make px4_sitl gz_x500` and **running it again**.

### **Full Reset (If the Issue Persists)**
If restarting doesn't help, try **cleaning the build environment** before rebuilding PX4:

```bash
make clean          # Remove compiled binaries but keep configuration files
make distclean      # Remove all compiled files and reset configuration
make px4_sitl       # Rebuild PX4 SITL
make px4_sitl gz_x500  # Start the simulation again
```
