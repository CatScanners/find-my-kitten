---
parent: Quick Start
title: Start
nav_order: 2
---

# Quick start

There is two ways to start developing this project further. First one is simpler and basically anybody can do it - a simulator. Second one is to build your own drone or use our existing setup, upload your code to it, and run test flights. In this page, both of these options are covered. Moreover, we have been working on a Docker development environment, information about that here as well.

## Simulator quick start (Windows or Linux)

### Requirements
Computer requirements? GPU for the simulator would be nice at least I reckon

### Setup toolchain

First, let's set up the toolchain:
1. Install the **PX4 toolchain** as per [PX4 Toolchain Guide](https://docs.px4.io/main/en/dev_setup/dev_env.html). If QGroundControl is not working for you, [here](https://www.youtube.com/watch?v=dQw4w9WgXcQ) guidance on how to proceed with the steps requiring QGroundControl. If there are any problems when installing PX4-Autopilot, refer to the Common problems -section.
2. Install **ROS2** and **Micro-XRCE** as per [PX4 ROS2 Guide](https://docs.px4.io/main/en/ros2/user_guide.html).
3. Use our [custom simulation](https://www.youtube.com/watch?v=dQw4w9WgXcQ) setup.
4. Clone our **find-my-kitten** repository:
`
git clone https://github.com/CatScanners/find-my-kitten
`
5. Make any changes to any of the packages inside **ros2_ws**-folder, or create new ones. Please refer to [ROS2 documentation](https://docs.ros.org/en/foxy/index.html). Shortly: change the code, `colcon build`, and `source install/setup.bash`. 

### Simulation startup 
1. Open up a **QGroundControl** window.
2. Run the **XRCE-agent**: `MicroXRCEAgent udp4 -p 8888`.
3. Start the simulation: 
- To start the simulation: `make px4_sitl gz_x500_baylands`
- To start the simulation camera bridge: `ros2 run ros_gz_image image_bridge /camera`.
4. From QGroundControl, **arm** and **takeoff**. If you don't have QGC, please refer to the Common problems -section.

### Actions startup
1. Start a node receiving offboard messages and then sending those periodically to PX4 simulator. `ros2 run px4_handler offboard_control`. By default just sends current coordinates.
2. Turn on offboard node from QGroundControl. Now the drone is flying with our ROS2 messges, currently just staying in its current position. Again, if you don't have QGC, please refer to the Common problems -section.
3. Star vision package to detect the ball.
`ros2 run vision_package object_detector.py --ros-args -p input_topic_name:="camera" -p output_topic_name:="detections"`
4. a - Start ballfinder node to start searching for the ball:
`ros2 run px4_handler ball_finder.py`, or b - Start predefined motions with two speeds: `ros2 run px4_handler hard_motions.py`.
5. a - With default code, the drone should search an area of 8m x 12m and stop and go a bit down when it sees a **sports ball**, or b - Does a few predefined motions in two different speeds.


## Real life quick start
1. Drone?
2. Jetson basebord initialization
3. Prepare for flight? :D
4. Find flight test site? :D
5. Apply for sum permissions? :D
6. Get certificate to fly?
7. Get a yellow west?
8. Fly!

## Docker setup
To be coming???