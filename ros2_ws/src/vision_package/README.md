Todo!
Documentation for vision_package

Image_transport needs to be installed separately eg: 
sudo apt install ros-humble-image-transport

Python scripts need to have exectution rights eg:
chmod +x object_recongizer.py

file header for the py file might be also needed: 
#!/usr/bin/env python3

Use numpy 1.24.1 if ROS2 package "CV_bridge" or others come up with errors about being buildt on numpy<2


Do not use pip packages of opencv pyhton. Apt works best
sudo apt install -y python3-opencv


How to use Docker container

docker build -t vision_package --build-arg DEPTHAI=true .

xhost +local:docker

docker run -it \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    vision_package

Check containers
docker ps -a

Start and attach stopped container
docker start [name]
docker attach [name]

Open a new shell to container
docker exec -it [name] /bin/bash

Nodes included in vision_package:

image_publisher

video_publisher.py

image_subscriber

object_recogniser.py

Ros2 base nodes + Depthai included nodes
