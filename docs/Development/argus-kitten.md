---
parent: Development
title: Documentation for argus-kitten
---

# Introduction
This package is a fork of
[isaac_ros_argus_camera](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera/index.html)
with a launch file (`isaac_ros_argus_camera_mono.launch.py`) that allows specifying the mode of the
camera to use (`v4l2-ctl --list-formats-ext` to see the modes). The default is to use mode 0, which
is the highest resolution. Since we use the camera mainly for YOLO which doesn't need such high
resolution, it is wasteful to have it publish it and use more power to transport the data.

## Running
To run the downwards camera, launch it with
```bash
ros2 launch argus-kitten isaac_ros_argus_camera_mono.launch.py
```

# Extra note
This could (and should, really) be refactored to just be a copy of the launch file with the
modifications made, and put in a package like `vision_package`.
