#!/bin/bash

start_nodes() {
    ros2 run vision_package image_publisher --ros-args -p pub_time:=0.1 &
    NODE1_PID=$!
    ros2 run vision_package object_detector.py &
    NODE2_PID=$!
    ros2 run vision_package object_tracker.py &
    NODE3_PID=$!
}

stop_nodes() {
    kill $NODE1_PID $NODE2_PID $NODE3_PID
    wait $NODE1_PID $NODE2_PID $NODE3_PID 2>/dev/null
    exit 0
}

# Trap Ctrl+C (SIGINT) to call the stop_nodes function
trap stop_nodes SIGINT

# Start the nodes
start_nodes

# Keep the script running to keep nodes alive
echo "Press Ctrl+C to stop all nodes."
while true; do
    sleep 1
done
