# Documentation for navigation_package
Quick and dirty documentation for navigation_package so yall maybe can use this too! 
## Nodes and quick explanations

Nodes included in navigation_package (where given ros arguments are the default values):

### Target deviation
Calculates the deviation of the target from the screen centre. Considers the screen center as origo and X, Y are +/- depending on which side they are.
```
ros2 run navigation_package target_deviation --ros-args \
    -p input_track:="tracked_objects_topic" \
    -p res_width:="1920" \ 
    -p res_height:="1080" \
    -p output_topic_name:="target_deviation_topic" 
```
