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

### Guide
```mermaid
    flowchart TD
    %%Style
        classDef A fill:green,stroke:black
        classDef D fill:grey,stroke:black

    subgraph Node_in_ROS2
        running_file_is_green.cpp:::A
        not_running_file_is_grey.py:::D
    end
    Node_in_ROS2 --publish--> topic --subscribe-->Node_in_ROS2
```

Green: Files that are included in CMakeList.txt.

Gray: Files that are not compiled in CMakeList.txt (legacy code, testing, debug, etc).
### Diagram

```mermaid
    flowchart TD

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

        subgraph px4_handler
            drone_movement_node
            offboard_control
        end
    
    %%Vision package
        
        subgraph image_publisher
            image_publisher.cpp:::A
            arducam_publisher.py:::A
            video_publisher.py:::A
        end

        subgraph image_subscriber
            image_subscriber.cpp:::A
        end

        subgraph object_detection_node
            ball_detector.py:::D
            object_detector.py:::A
        end

        subgraph object_tracker_node
            object_tracker.py:::A
        end

        subgraph video_recorder
            record_video_from_rosbag.py:::D
        end

        subgraph tracked_image_publisher
            track_publisher.py:::A
        end

    %%ball_finder.py

        drone_movement_node -->/custom_trajectory

        drone_movement_node -->/fmu/in/vehicle_command

        /fmu/out/vehicle_local_position --> drone_movement_node

        /detections --> drone_movement_node

    %%offboard_control.cpp

        offboard_control --> /fmu/in/offboard_control_mode

        offboard_control --> /fmu/in/trajectory_setpoint

        offboard_control --> /fmu/in/vehicle_command

        /custom_trajectory--> offboard_control

        /fmu/out/vehicle_local_position -->offboard_control

    %%image_publisher

        image_publisher --> /image_topic

    %%image_subscriber

        /image_topic --> image_subscriber

    %%object_detection_node

        /image_topic --> object_detection_node
        
        object_detection_node --> /detected_objects_topic

       object_detector.py --> /image_dimension_topic

       object_detector.py --> /detections

    %%object_tracker_node

        /detected_objects_topic --> object_tracker_node

        object_tracker_node --> /tracked_objects_topic

    %%video_recorder
        /image_topic --> video_recorder 

    %%tracked_image_publisher

        /image_topic --> tracked_image_publisher

        /tracked_objects_topic --> tracked_image_publisher

        tracked_image_publisher --> /tracked_image_topic

    %%Other components

        gazebo?((simulation_image_bridge)) --/camera--> object_detector.py

        camera((Camera)) --/dev/video0 --> image_publisher.cpp

        camera((Camera)) --/dev/video0 --> arducam_publisher.py

        web((Web)) --> video_publisher.py

```