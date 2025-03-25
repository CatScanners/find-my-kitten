# Documentation for navigation_package
Quick and dirty documentation for navigation_package so yall maybe can use this too! 
## Nodes and quick explanations

Nodes included in navigation_package (where given ros arguments are the default values):

### Target deviation
Calculates the deviation of the target from the screen centre. Considers the screen center as origo and X, Y are +/- depending on which side they are. Needs image_dimension_topic for X&Y calculations and supports a depth distance to target. (In 3D space calculates the deviation in manner as to be 1000 units away. badly worded, ask Kaius :D )
```
ros2 run navigation_package target_deviation --ros-args \
    -p input_track:="tracked_objects_topic" \
    -p image_dimension_topic:="image_dimension_topic \
    -p depth_topic:="depth_data_topic" \ 
    -p output_topic_name:="target_deviation_topic" 
```
