# PX4 handler

This package is for the code that handles the communication between PX4 and companion computer.
Currently has two scripts: hard_motions.py and offboard_control.cpp. First one has 
some hardcoded offboard movements, second one works as a bridge between PX4 and Jetson 
that handles tracepoints periodically to the Pixhawk. 

## PREREQ: Run XRCE bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

## PREREQ: Run Gazebo simulation
```bash
cd xxx/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 # headless if you don't have gui
```

## PREREQ: Build PX4_ros_com package
```bash
colcon build --packages-select px4_handler
```

## PREREQ: Source setup files
```bash
source install/setup.bash
```

## Run flight example (works similarly with simulator as well)
1. Disable the safe mode (not needed if connected to QGC!).
```bash
mavproxy.py --master=udp:127.0.0.1:14540
param set COM_FLTMODE2 7 # 7 = Offboard, as mentioned here https://docs.px4.io/main/en/advanced_config/parameter_reference.html#commander
```
2. Arm via QGroundControl.
3. Switch to offboard mode via QGroundControl.
4. Offboard control node
```bash
ros2 run px4_handler offboard_control
```
5. Start motions
```bash
ros2 run px4_ros_com hard_motions.py
```