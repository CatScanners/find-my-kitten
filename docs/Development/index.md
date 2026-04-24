---
title: Development
nav_order: 5

---

# `ros2_ws` packages
The main packages are `vision_package` and `visual_navigation`

## vision_package
Includes nodes for running image detection and debugging related to it. The main node is
`object_detector.py`, which runs YOLO tracking and detection on an image topic and publishes its
detections. Check the dedicated page for more.

## visual_navigation
Nodes related to localisation of the drone based on camera feeds. Currently has VSLAM and our custom
point odometry algorithm. Check the dedicated page for more.

## argus-kitten
Forked from isaac_ros_argus_camera from NVIDIA to allow specifying the camera mode to use. Check the
'argus-kitten' page for more.

## isaac_ros_common
Another fork from NVIDIA's isaac_ros_common repository. Our changes have been mostly to improving
the devcontainer tooling it has. Check the 'Development container' page for more info on changes

## kitten_sim
Things related to running IsaacSIM

## px4_handler
Mostly used by the 2024 team for the `ball_finder.py` node. Also includes `offboard_control` node
for sending commands to the drone
