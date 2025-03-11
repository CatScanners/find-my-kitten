#!/bin/bash

TOPICS=(
    "/fmu/out/vehicle_odometry"
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_global_position"
    "/fmu/out/sensor_combined"
    "/fmu/out/vehicle_attitude"
    "/fmu/out/vehicle_gps_position"
    "/zed_node/left_raw/image_raw_color"
    "/zed_node/right_raw/image_raw/_color
    "/zed_node/left_gray/image_rect_gray"
    "/zed_node/right_gray/image_rect_gray"
    "/zed_node/left/image_rect_color"
    "/zed_node/right/image_rect_color"
    "/zed_node/left_gray/image_rect_gray"
    "/zed_node/right_gray/image_rect_gray"
    "/zed_node/left/image_rect_color"
    "/zed_node/right/image_rect_color"
    "/zed_node/left_gray/camera_info"
    "/zed_node/right_gray/camera_info"
)


ros2 bag record "${TOPICS[@]}"
