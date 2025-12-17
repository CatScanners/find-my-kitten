#!/bin/bash

SESSION="offboard_control"

tmux new-session -d -s $SESSION

# Pane 0
tmux send-keys -t $SESSION:0.0 'bash -c "source install/setup.bash && ros2 run vision_package arducam_publisher.py"' C-m

# Pane 1
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION:0.1 'bash -c "source install/setup.bash && ros2 run px4_handler ball_finder.py"' C-m

# Pane 2
tmux select-pane -t $SESSION:0.1
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION:0.2 'bash -c "source install/setup.bash && ros2 run px4_handler offboard_control"' C-m

# Pane 3
tmux select-pane -t $SESSION:0.2
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION:0.3 'bash -c "source install/setup.bash && ros2 run vision_package object_detector.py --ros-args -p output_topic_name:=\"detections\""' C-m


# Pane 3
#tmux select-pane -t $SESSION:0.3
#tmux split-window -v -t $SESSION
#tmux send-keys -t $SESSION:0.4 'bash -c "source install/setup.bash && ros2 run rqt_image_view rqt_image_view"' C-m

# Layout
tmux select-layout -t $SESSION tiled

tmux attach-session -t $SESSION

