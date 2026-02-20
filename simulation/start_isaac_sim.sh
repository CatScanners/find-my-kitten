#!/usr/bin/env bash

# Check if both python file arguments are provided
if [ $# -lt 2 ]; then
    echo "Usage: $0 <px4_handler_script> <isaac_sim_script>"
    echo "Example: $0 ball_finder.py first_standalone.py"
    exit 1
fi

PX4_SCRIPT="$1"
ISAAC_SCRIPT="$2"

ROS_DISTRO=${ROS_DISTRO:-humble}

# Resolve directory of this script (handles symlinks too)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Kill existing session if any
tmux kill-session -t kitten_sim 2>/dev/null || true

# Start base and split 5 times (creates 6 panes, no targets needed)
tmux new-session -d -s kitten_sim "bash -lc \"source /opt/ros/$ROS_DISTRO/setup.bash; cd '$PROJECT_ROOT/ros2_ws'; source '$PROJECT_ROOT/.venv/bin/activate'; source install/setup.bash; exec MicroXRCEAgent udp4 -p 8888 -v 4\""

tmux split-window -h "bash -ic \"cd '$PROJECT_ROOT/simulation/IsaacSim/standalones'; isaac_run $ISAAC_SCRIPT\""

tmux split-window -v "bash -lc \"source /opt/ros/$ROS_DISTRO/setup.bash; cd '$PROJECT_ROOT/ros2_ws'; source '$PROJECT_ROOT/.venv/bin/activate'; source install/setup.bash; exec ros2 run px4_handler offboard_control\""

tmux split-window -h "bash -lc \"echo 'Waiting for Isaac Sim to load...'; sleep 30; source /opt/ros/$ROS_DISTRO/setup.bash; cd '$PROJECT_ROOT/ros2_ws'; source '$PROJECT_ROOT/.venv/bin/activate'; source install/setup.bash; exec ros2 run px4_handler $PX4_SCRIPT\""

tmux split-window -v "bash -lc \"source /opt/ros/$ROS_DISTRO/setup.bash; cd '$PROJECT_ROOT/ros2_ws'; source '$PROJECT_ROOT/.venv/bin/activate'; source install/setup.bash; exec ros2 run vision_package image_subscriber --ros-args -r /image_topic:=/drone1/down_facing_camera/color/image_raw\""

tmux split-window -h "bash -lc \"source /opt/ros/$ROS_DISTRO/setup.bash; cd '$PROJECT_ROOT/ros2_ws'; source '$PROJECT_ROOT/.venv/bin/activate'; source install/setup.bash; exec ros2 run vision_package object_detector.py --ros-args -p input_topic_name:='/drone1/down_facing_camera/color/image_raw' -p output_topic_name:='detections'\""

# Apply layouts for 3x2 equal grid
tmux select-layout -t kitten_sim tiled

tmux attach-session -t kitten_sim
