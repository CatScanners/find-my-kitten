# Software architecture diagram

This Markdown file contains different architecture diagrams for the project, and it should replace some outdated documentation by the previous team (CatScanner 2024-2025). 
Currently, we use [Markdown Preview Mermaid Support by Matt Bierner](https://marketplace.visualstudio.com/items?itemName=bierner.markdown-mermaid) to simplify the creation process. After installing the extension, the diagram will be shown in the preview.

If you cannot install the extension for some reason, you can download another Mermaid-compatible extension and modify the code manually to match the format.

If you are new to Mermaid and want to get familiar with it, please check the [Youtube video by PetterTech](https://www.youtube.com/watch?v=qGsQolMh9zE), [Youtube video by Red Gregory](https://www.youtube.com/watch?v=-XV1JBfhgWo&list=PLw5h0DiJ-9PC49ItLKBQlWSMH1zolOCXS), or [The official documentation by mermaid.js.org](https://mermaid.js.org/syntax/flowchart.html).

## Changelog

| Date | Changes |
| :-- | :-- |
| 2026-01-09 | Add ROS2 node architecture diagram |

## 1. Relationship between different ROS2 nodes

Green: Files that are included in CMakeList.txt.

Gray: Files that are not compiled in CMakeList.txt (legacy code, testing, debug, etc).
### Diagram

```mermaid
    flowchart TB
    %%Style for color
        %% Activated
        classDef A fill:green,stroke:black

        %% Deactivated
        classDef D fill:grey,stroke:black

    %%PX4_handler
        subgraph drone_movement_node
            ball_finder.py:::A
        end

        subgraph offboard_control
            offboard_control.cpp:::A
        end

    %%Vision package
        
        subgraph argus-kitten
            isaac_ros_argus_camera_mono:::A
        end

        subgraph object_detection_node
            object_detector.py:::A
        end

    %%ball_finder.py

        drone_movement_node --movement commands -->/custom_trajectory

        drone_movement_node -->/fmu/in/vehicle_command

        /fmu/out/vehicle_local_position --> drone_movement_node

        /detections --> drone_movement_node

    %%offboard_control.cpp

        offboard_control --enable offboard mode --> /fmu/in/offboard_control_mode

        offboard_control --movement commands --> /fmu/in/trajectory_setpoint

        /custom_trajectory--> offboard_control

        /fmu/out/vehicle_local_position -->offboard_control

    %%argus-kitten

        argus-kitten --video feed --> /argus/left/image_raw

    %%object_detection_node

        /argus/left/image_raw --> object_detection_node

       object_detector.py --detected objects --> /detections

    %% Point odometry
        subgraph point_odometry
            point_odometry.cpp:::A
        end

        /argus/left/image_raw --> point_odometry
        point_odometry --> /fmu/in/vehicle_visual_odometry

    %%Stereo camera
        subgraph stereo_node
            stereo_publisher.cpp:::A
        end

        subgraph stereo_topics["Stereo Topics"]
            topics["
                /left/image_rect
                /left/camera_info
                /right/image_rect
                /right/camera_info
                /imu
                /tf
                /tf_static
            "]
        end
        stereo_publisher.cpp --camera feed --> stereo_topics

    %%VSLAM
        subgraph visual_slam_node
            isaac_ros_visual_slam:::A
        end
        stereo_topics --> isaac_ros_visual_slam
        isaac_ros_visual_slam --pose estimates --> /visual_slam/tracking/odometry
        /visual_slam/tracking/odometry --vslam_message_transform.cpp --> /fmu/in/vehicle_visual_odometry
        /fmu/in/vehicle_visual_odometry --> ekf2

    %%Cameras
        camera((Downward Camera)) --/dev/video0 --> isaac_ros_argus_camera_mono
        
        stereo_camera((OAK-D Pro Stereo Camera)) --/dev/bus/usb --> stereo_publisher.cpp

    %% Flight control / PX4
        subgraph flight_controller_inputs["Flight Controller Inputs"]
            IMU
            Barometer
            Magnetometer
        end
        px4(("Flight Controller (PX4)")) --measured data --> flight_controller_inputs
        flight_controller_inputs --> ekf2[EKF2]:::A
        ekf2 --position estimates --> /fmu/out/vehicle_local_position
        /fmu/in/offboard_control_mode --> px4
        /fmu/in/trajectory_setpoint --> px4
        /fmu/in/vehicle_command --> px4

```
