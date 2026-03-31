# shellcheck shell=bash
setup_script=/workspaces/isaac_ros-dev/ros2_ws/install/setup.bash

if [[ -f "$setup_script" ]]; then
    # shellcheck disable=SC1090
    source "$setup_script"
fi
