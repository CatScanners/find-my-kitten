#!/bin/bash

TOPICS=(
    "/fmu/out/estimator_status_flags"
    "/fmu/out/failsafe_flags"
    "/fmu/out/manual_control_setpoint"
    "/fmu/out/position_setpoint_triplet"
    "/fmu/out/sensor_combined"
    "/fmu/out/timesync_status"
    "/fmu/out/vehicle_attitude"
    "/fmu/out/vehicle_command_ack"
    "/fmu/out/vehicle_control_mode"
    "/fmu/out/vehicle_gps_position"
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_odometry"
    "/fmu/out/vehicle_status"
    "/zed/zed_node/left_raw_gray/image_raw_gray"
    "/zed/zed_node/right_raw_gray/image_raw_gray"
    "/zed/zed_node/left_gray/camera_info"
    "/zed/zed_node/right_gray/camera_info"
    "/image_topic"
)


source /home/baseboard/ws_sensor_combined/install/setup.bash && ros2 bag record "${TOPICS[@]}"