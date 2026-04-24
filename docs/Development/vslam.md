---
parent: Development
title: VSLAM
---

# Introduction
The VSLAM package we use is
[isaac_ros_visual_slam](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
from NVIDIA (specifically version 3.2). This documentation focuses mainly on how we have used it and
our experiences with it, rather than the details of how it works.

## Input topics
In addition to the subscribed topics mentioned in the documentation, it also needs the `/tf_static`
topic. This topic includes static transformations that describe how the camera is built, i.e. how far
are the two cameras, where the IMU is in the camera, etc. The transforms are organized as a tree
with a single frame at its root, such that you can get the transform of any frame to any other frame
via following the tree. The transforms are published when using the OAK-D Pro stereo camera by the
`stereo.launch.py` launch file (or the `stereo_vslam.launch.py` which also launches it).

One thing in the static transforms you may want to change is the transform from the whole camera
itself to the drone's frame. This is published as the transform from the frame `oak-d_frame`, which
all other frames converge to, to the frame `oak-d-base-frame`, which is the root of the tree. You
can change the trasform by modifying the `cam_pos_*` and `cam_{roll,pitch,yaw}` (which takes
radians) variables in `stereo.launch.py`. We mainly used this to test the camera at different
angles, e.g., facing 45° down.

Also note that the `vslam.launch.py` launch file remaps some topics to match what VSLAM subscribes
to.

## Running it
We have a launch file in our `visual_navigation` package, which launches the VSLAM node with the
parameters specified in the launch file. In addition, we have a launch file to launch VSLAM along
with the stereo camera for ease of use. For a list of parameters and what they do, check the
IsaacROS package's documentation in the link above. For modifying the parameters, we generally just
edit the launch file and rebuild the package as it's quite fast, but adding parameters passable via
the CLI is possible.

The package also includes the `vslam_message_transform` node, subscribed to the output of VSLAM,
transforms the message into a passable type for PX4's EKF2, and publishes it to
`/fmu/in/vehicle_visual_odometry`, so that EKF2 can use it according to `EKF2_EV_CTRL` PX4
parameter.

## Analyzing the output
To see how VSLAM performs, you can use [rviz2](https://github.com/ros2/rviz). Simply start up rviz2
while VSLAM is publishing its output (or it's being played back from a ros bag).
```bash
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz
```

## Replaying VSLAM from a ROS bag
Doing this allows you to replay VSLAM with different parameters. To replay VSLAM you just have to
start the VSLAM node via `vslam.launch.py` while playing a bag that has the right input topics. You
also need to make sure you don't play any interfering topics simultaneously, e.g., having the stereo
camera publishing live or playing the output of the VSLAM from a bag. You can play only wanted
topics from a bag with the `--topics` flag.
```bash
ros2 bag play <bag> \
  --topics /imu /tf_static /left/image_rect /left/camera_info /right/image_rect /right/camera_info
```

### Replaying with a different `/tf_static`
This could be necessary if you happen to record a ros bag of a flight while publishing the wrong
`/tf_static` from the camera. In this case you need to not publish `/tf_static` from the bag and
instead publish the right one yourself. What we've done is just comment out the part in
`stereo.launch.py` that starts up the camera, keeping the part that publishes the transforms. Then
you can edit the `cam_*` variables in the launch file and launch it (remember to rebuild and
source!).
