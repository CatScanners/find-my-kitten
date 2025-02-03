#!/bin/bash
# After building and sourcing, this can be used to run both nodes in a single terminal. Testing purposes. 
ros2 run vision_package image_publisher &
ros2 run vision_package object_recognizer.py
