# Build and run py_pubsub-example

## Check dependencies
```bash
rosdep install -i --from-path src --rosdistro humble -y & rosdep init & rosdep update
```

## Build code
```bash
colcon build --packages-select <pckg-name>
```
 

## Source setup files (new terminal?)
```bash
source install/setup.bash
```

## Run talker & listener (or just your node)
```bash
ros2 run <pckg-name> talker
```
```bash
ros2 run <pckg-name> listener
```

## Run flight example
First, start to track telemetry to verify results later.
```bash
ros2 run px4_ros_com vehicle_gps_position_listener
```

Disable the safe mode.
```bash
mavproxy.py --master=udp:127.0.0.1:14540
param set COM_FLTMODE2 7 # 7 = Offboard, as mentioned here https://docs.px4.io/main/en/advanced_config/parameter_reference.html#commander
```

Run example code that should raise the drone to 500m.
```bash
ros2 run px4_ros_com offboard_control
```