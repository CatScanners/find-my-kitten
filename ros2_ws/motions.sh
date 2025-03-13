#!/bin/bash

# Define a function to publish the trajectory messages
publish_trajectory() {
  ros2 topic pub /custom_trajectory px4_msgs/msg/TrajectorySetpoint \
    "{ position: [ $1, $2, $3 ], yaw: $4 }" -1
}

# Define motion patterns (velocity_x, velocity_y, velocity_z, yaw)
motions=(
  "0.0 0.0 -3.0 0.0"      # Up
  "0.0 0.0 -5.0 0.0"     # Down
  "8.0 0.0 -5.0 0.0"      # Forward
  "0.0 0.0 -5.0 0.0"     # Backward
  "0.0 8.0 -5.0 0.0"      # Sideways Right
  "0.0 0.0 -5.0 0.0"     # Sideways Left
  "0.0 0.0 -5.0 3.14"     # Rotate (yaw = 180 degrees)
)

# Repeat each motion 4 times
for i in {0..6}; do
    # Get motion parameters (velocity_x, velocity_y, velocity_z, yaw)
    IFS=' ' read -r velocity_x velocity_y velocity_z yaw <<< "${motions[$i]}"
    
    # Publish trajectory for the current motion
    echo "Publishing motion $((i+1))"
    publish_trajectory "$velocity_x" "$velocity_y" "$velocity_z" "$yaw"
    
    sleep 10
done

echo "Drone movement complete!"