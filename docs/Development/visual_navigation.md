# Documentation for visual_navigation

## Introduction

Visual navigation is the package responsible for stereo camera, Isaac ROS VSLAM, and point odometry. It contains multiple launch files for launching different configurations. The package integrates visual SLAM for odometry estimation and stereo vision for depth perception, bridging to PX4 for drone navigation.

## Nodes and their short explanations

Nodes included in visual_navigation:

### stereo_publisher
Custom wrapper for the OAK-D pro camera. The default wrappers had all sorts of unnecessary features running which caused a brownout so we wrote a minimalistic wrapper (brownout could also be solved by using the Y adapter, but we did not do that). Publishes stereo camera data including rectified left and right images, disparity/depth maps, and IMU data using the DepthAI (Luxonis) OAK-D camera. This node sets up a pipeline for stereo depth processing and publishes ROS2 messages for further processing. For the full list of launch parameters and other examples and API documentation refer to https://docs.luxonis.com/software/ros/depthai-ros/ (also includes how to use the other features of the camera)

### vslam_message_transform
Transforms odometry messages from Isaac ROS Visual SLAM to PX4 VehicleOdometry format. It subscribes to the vSLAM odometry topic, performs coordinate frame transformations (from vSLAM axes to PX4 FRD), and publishes the transformed odometry for PX4's EKF2. This node has no configurable parameters and uses default topics: subscribes to "/visual_slam/tracking/odometry" and publishes to "/fmu/in/vehicle_visual_odometry". For more info on the VSLAM visit https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html 

### point_odometry
Converts tracked object detections into vehicle odometry messages. This node processes Detection2DArray messages, extracts tracked objects, and computes odometry based on their positions. Note: The actual odometry calculation logic is currently a placeholder and needs implementation. Subscribes to "detections" topic (vision_msgs/Detection2DArray) and publishes to "point_odometry" topic (px4_msgs/VehicleOdometry).

## Launch Files

The package includes several launch files for different configurations. The launch files don't contain the full lists of parameters, for those consult the oness listed from the links above.

### stereo.launch.py
Launches the stereo_publisher node with configurable parameters for the stereo camera setup. Example usage:
```
ros2 launch visual_navigation stereo.launch.py
```

### stereo_vslam.launch.py
Launches both stereo_publisher and vSLAM components together for integrated stereo vision and SLAM, including the vslam_message_transform node. This is the one used when gathering data while flying.

### vslam.launch.py
Launches the vSLAM pipeline without stereo publisher, assuming camera input from elsewhere, and includes the vslam_message_transform node. This can be used for example when playing back a rosbag of a recorded flight data of depth camera.

## Notes and debugging details

- Ensure DepthAI library and dependencies are installed for stereo_publisher.
- Coordinate transformations in vslam_message_transform rotate from vSLAM (x=forward, y=left, z=up) to PX4 FRD (x=forward, y=right, z=down).
- point_odometry node requires implementation of the odometry calculation logic (currently placeholder).


