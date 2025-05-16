---
parent: Introduction
title: Quick Start
nav_order: 2
---

# Quick start

There is two ways to start developing this project further. First one is simpler and basically anybody can do it - a simulator. Second one is to build your own drone or use our existing setup, upload your code to it, and run test flights. In this page, both of these options are covered. Moreover, we have been working on a Docker development environment, information about that here as well.

## Nvidia Jetson initialization
Please refer to our [Jetson Orin setup guide](https://catscanners.github.io/find-my-kitten/Jetsons%20&%20Pixhawk/jetson-setup.html).

## Simulator quick start (Windows or Linux)

This part guides the user on how to install a simulator, ROS2, find-my-kitten repository, QGroundControl, and run our main nodes on the system of consisting this software.

### Requirements
- A Windows or a Linux computer, preferably with GPU since the simulation is quite a heavy software.
- If you are already using a Nvidia Jetson -computer, please refer to the [Nvidia Jetson initialization](#jetson-initialization).
- Basic knowledge on ROS2. Please refer to our [ROS2 guide](https://catscanners.github.io/find-my-kitten/Jetsons%20&%20Pixhawk/ROS2%20Compiled%20Guide.html).

### Setup toolchain

First, let's set up the toolchain:
1. Install the **PX4 toolchain** as per [PX4 Toolchain Guide](https://docs.px4.io/main/en/dev_setup/dev_env.html). If QGroundControl is not working for you, [here](https://catscanners.github.io/find-my-kitten/QGroundControl%20&%20Drone/) guidance on how to proceed with the steps requiring QGroundControl. If there are any problems when installing PX4-Autopilot, refer to the Common problems -section.
2. Install **ROS2** and **Micro-XRCE** as per [PX4 ROS2 Guide](https://docs.px4.io/main/en/ros2/user_guide.html).
3. Use our [custom simulation markdown](https://github.com/CatScanners/find-my-kitten/blob/main/simulation/instructions.md) / [custom simulation website documentation](https://catscanners.github.io/find-my-kitten/Simulation%20&%20flight%20analysis/Simulation%20setup.html) setup.
4. Clone our **find-my-kitten** repository:
`
git clone https://github.com/CatScanners/find-my-kitten
`
5. Make any changes to any of the packages inside **ros2_ws**-folder, or create new ones. Please refer to [ROS2 documentation](https://docs.ros.org/en/humble/index.html). Shortly: change the code, `colcon build`, and `source install/setup.bash`. 

### Simulation startup 
1. Open up a **QGroundControl** window.
2. Run the **XRCE-agent**: `MicroXRCEAgent udp4 -p 8888`.
3. Start the simulation: 
- To start the simulation: `make px4_sitl gz_x500_baylands`
- To start the simulation camera bridge: `ros2 run ros_gz_image image_bridge /camera`.
4. From QGroundControl, **arm** and **takeoff**. If you don't have QGC, please refer to the Common problems -section.

### Machine vision startup
1. IMPORTANT. On drone startup make sure that all connected cameras are visible. This can be done with command `ls /dev/ | grep video`. If you have only the arducam globalshutter camera connected, you should only see /dev/Video0. 
If you've connected the Zed depth camera, you should see three cameras: /dev/Video0 ...Video1 and ...Video2 (Depth camera has two video feeds). If you only see two or no cameras, powercycle the drone. The arducam uses custom drivers which if not loaded properly means it will not show up nor work.  
2. Build and source our ros2 packages. Otherwise `ros2 run vision_package ...` will result in package not found. To build run `colcon build` and after building run `source install/setup.bash`. This should be done inside folders `ros2_ws` or `ros2_ws/src` to build all packages. 
3. Run the camera publishing node. For a regular USB / CSI camera use `ros2 run vision_package image_publisher` and if you are using an arducam globalshutter camera run `ros2 run vision_package arducam_publisher.py`. These fill publish video frames to a ros2 topic called image_topic. You can check if the node is working and topic is visible with `ros2 topic list`.
4. Run the object detection node. Command compatible with offboard_control is `ros2 run vision_package object_detector.py --ros-args -p input_topic_name:="camera" -p output_topic_name:="detections"`.
5. To preview the image stream you can run `ros2 run vision_package image_subscriber` node. This brings up an window which displays the video feed from topic `image_topic`.

### Actions startup
1. Start a node receiving offboard messages and then sending those periodically to PX4 simulator. `ros2 run px4_handler offboard_control`. By default just sends current coordinates.
2. Turn on offboard node from QGroundControl. Now the drone is flying with our ROS2 messges, currently just staying in its current position. Again, if you don't have QGC, please refer to the Common problems -section.
3. If you haven't already, star vision package to detect the ball.
`ros2 run vision_package object_detector.py --ros-args -p input_topic_name:="camera" -p output_topic_name:="detections"`
4. a - Start ballfinder node to start searching for the ball:
`ros2 run px4_handler ball_finder.py`, or b - Start predefined motions with two speeds: `ros2 run px4_handler hard_motions.py`.
5. a - With default code, the drone should search an area of 8m x 12m and stop and go a bit down when it sees a **sports ball**, or b - Does a few predefined motions in two different speeds.


## Real life quick start (Jetson baseboard + PX6 + drone)

This part guides the user on how to setup the actual drone and how to connect a Holybro Pixhawk Jetson Baseboard into it, the installation of all the relevant software. If you already have our pre-built drone with the baseboard, feel free to skip until the "Running the software part". This part also includes information about necessary permissions and who is actually allowed to fly the drone in Finland.

### Requirements
- Knowledge of Linux.
- [Holybro Pixhawk Jetson Baseboard](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html)
- Drone frame
- Something else?

### Setting up the Holybro Pixhawk Jetson Baseboard

To set up the baseboard, please follow this [guide](https://catscanners.github.io/find-my-kitten/Jetsons%20&%20Pixhawk/jetson-setup.html) of ours. It has a link to the official guide on setting up the baseboard, and also the deviations from the official material. It also includes steps for initializing a Jetson Orin Nano Devkit, which might be helpful.

### Setting up the drone

Assembly follows the [HolyBro X500v2 guide](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2) with Jetson and Pixhawk setup as mentioned above.

### Fly in real life

1. Build the drone, refer to the [Setting up the drone](#setting-up-the-drone).
2. Set up the Holybro Pixhawk Jetson Baseboard that is on the drone with our [guide](#setting-up-the-holybro-pixhawk-jetson-baseboard)
3. Do all the real-life overhead related to flying a drone:
- Get relevant permissions to fly.
- Setup radio connection and buy a controller.
- Find a test site.
- Ensure a good test day weather.
- Most of this stuff is probably taught at a drone flying certificate school.
- Go through the [flight checklist](../assets/Flight-checklist.pdf) provided by our project.
- Use our [flight plan template](../assets/MissionPlanTemplate.docx) to prepare for the flight.
4. Arm, takeoff and fly with position/altitude mode in QGroundControl with your controller. Then, switch to offboard mode and run the same scripts as with [actions startup](#actions-startup).

## Docker setup

We provide a Dockerfile and instructions for the simulation [here](https://github.com/CatScanners/find-my-kitten/blob/main/simulation/instructions.md). The section in Isaac ROS in the [Jetson Orin setup guide](https://catscanners.github.io/find-my-kitten/Jetsons%20&%20Pixhawk/jetson-setup.html) further covers Docker usage elsewhere.
