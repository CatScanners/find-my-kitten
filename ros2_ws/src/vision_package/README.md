# Documentation for vision_package
Quick and dirty documentation for vision_package so yall maybe can use this too! 
## Nodes and quick explanations

Nodes included in vision_package:

### image_publisher
Publishes regular webcam from /dev/video0 as images in the topic **/image_topic** . Video number can be configured from variables.hpp file in vision_package/include/variables.hpp . 
``` 
ros2 run vision_package image_publisher
```

### video_publisher.py
Publishes video or images from local files or links. Online videos are youtube and example of specifying a source can be seen below.  
``` 
ros2 run vision_package video_publisher.py --ros-args -p input_source:="LINK/PATH"
```
### image_subscriber
Subscribes to the topic to which image_publisher publishes images. This nodes opens a window in which it displays images it receives. 
``` 
ros2 run vision_package image_subscriber
```
### object_detector.py
Ultralytics pretrained yolov5 model based object detector node. Listens to /image_topic for input. Outputs yolo bounding box positions in terminal and displays a window where the image and rendered bounding boxes can be seen. 
``` 
ros2 run vision_package object_detector.py
```
### Depthai-ros included nodes
DepthAI-ROS is a optional include  in vision_package Dockerfile. Documentaion of it is poor but node names are descriptive enough and can be seen here: https://github.com/luxonis/depthai-ros/tree/humble .
Using Depthai-ros example RGB camera node:
```
ros2 run depthai_examples rgb_stereo_node
```

## How to use Docker container

Build docker image vision_package 
```
docker build -t vision_package --build-arg DEPTHAI=true .
```
Allow X11 forwarding if graphics needed inside the container
```
xhost +local:docker
```
Run the vision_package image to create a container
```
docker run -it \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    vision_package
```
Check containers to see the **NAME** or id of just created container 
```
docker ps -a
```
Start and attach stopped container
```
docker start NAME
docker attach NAME
```

Open a new shell to container
```
docker exec -it NAME /bin/bash
```
Some housekeeping commands \
Remove container
```
docker rm NAME
```
Remove Image
```
docker image rm vision_package
```

## How to use in host enviroment?

Follow and run the dockerfile CMD commands. Dockerfile uses Ubuntu 22.04 and you can follow it as an instruction set.  


## Notes and debuggin details
Mostly problems I've ran across and found some fix

Image_transport needs to be installed separately eg: 
sudo apt install ros-humble-image-transport

Python scripts need to have exectution rights eg:
chmod +x object_recongizer.py

file header for the py file might be also needed: 
#!/usr/bin/env python3

Use numpy 1.24.1 if ROS2 package "CV_bridge" or others come up with errors about being buildt on numpy<2

Do not use pip packages of opencv pyhton. Apt works best
sudo apt install -y python3-opencv
