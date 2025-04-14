---
parent: Software setup
---
# Find-my-kitten ROS offboard pipeline


## Run XRCE bridge
```bash
MicroXRCEAgent udp4 -p 8888 # in PX4-Autopilot directory
```


## Run predefined motion (up, left, forward, rotate) x2 (in simulator or live):
1. Start offboard node
```
ros2 run px4_handler offboard_control # start offboard node
```
This node first starts to print out relevant local position information, and starts to send hardcoded (relative) tracepoints to the starting position, which is now set to 0.0, 0.0, -5.0. 


2. Arm from QGroundCtonrol / via mavproxy by running ```arm throttle```.
3. Set to offboard node from QGroundControl 
4. Record
```
ros2 bag record /fmu/out/vehicle_local_position
```
5. Start motions
```
ros2 run px4_handler maneuver.py # start the script
```
This node waits for 5s and then starts to send the motion tracepoints.

## Ball rescuer
- *Offboard node* = Initially sends current coordinates to PX4 to keep the offboard mode on and the drone in the current location.
- *Ball finder* = Goes through an area. If vision_package finds a sports ball (yolov5 class_id 32), stops all current motions and descents to 5m.
- *Vision_package* = Analyzes the camera footage and sends a message to a topic if anything is detected.

## Run ball rescuer (go through an area and descent to 5m if a ball is found)
1. Start offboard node
```
ros2 run px4_handler offboard_control
```

2. Star vision package to detect nodes
```
ros2 run vision_package object_detector.py --ros-args -p input_topic_name:="camera" -p output_topic_name:="detections"
```

3. Start ballfinder node
```
ros2 run px4_handler ball_finder.py
```
