#!/bin/bash

TOPICS=(
    "/fmu/out/vehicle_odometry"
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_global_position"
    "/fmu/out/sensor_combined"
    "/fmu/out/vehicle_attitude"
    "/fmu/out/vehicle_gps_position"
)


ros2 bag record "${TOPICS[@]}"
