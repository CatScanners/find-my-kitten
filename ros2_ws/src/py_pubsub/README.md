# Build and run py_pubsub-example

## Check dependencies
```bash
rosdep install -i --from-path src --rosdistro jazzy -y & rosdep init & rosdep update
```

## Build code
```bash
colcon build --packages-select py_pubsub
```


## Source setup files (new terminal?)
```bash
source install/setup.bash
```

## Run talker & listener
```bash
ros2 run py_pubsub talker
```
```bash
ros2 run py_pubsub listener
```
