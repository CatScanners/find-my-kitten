#!/bin/bash

TOPICS=(
    "/fmu/out/vehicle_odometry"
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_global_position"
    "/fmu/out/sensor_combined"
    "/fmu/out/vehicle_attitude"
    "/fmu/out/vehicle_gps_position"
    "/zed_node/left_raw_gray/image_raw_gray"
    "/zed_node/right_raw_gray/image_raw_gray"
    "/zed_node/left_gray/camera_info"
    "/zed_node/right_gray/camera_info"
)


ros2 bag record "${TOPICS[@]}"
