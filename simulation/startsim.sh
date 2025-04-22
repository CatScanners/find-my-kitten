#!/bin/bash

# Start a tmux session named 'sim'
tmux new-session -d -s sim

# Pane 0: QGroundControl
tmux send-keys -t sim:0 'cd /find-my-kitten && ./QGroundControl.AppImage' C-m

# Pane 1: ROS-GZ image bridge
tmux split-window -h -t sim
tmux send-keys -t sim:0.1 'source /opt/ros/humble/setup.bash && ros2 run ros_gz_image image_bridge /camera' C-m

# Pane 2: MicroXRCEAgent
tmux split-window -v -t sim:0.1
tmux send-keys -t sim:0.2 'cd /find-my-kitten/ros2_ws/src/simulation/Micro-XRCE-DDS-Agent/build/ && MicroXRCEAgent udp4 -p 8888' C-m

# Pane 3: PX4 simulation
tmux select-pane -t sim:0.0
tmux split-window -v
tmux send-keys -t sim 'cd /find-my-kitten/ros2_ws/src/simulation/PX4-Autopilot && make px4_sitl gz_x500_gimbal_baylands' C-m

# Attach to the session
tmux select-pane -t sim:0.0
tmux attach-session -t sim
