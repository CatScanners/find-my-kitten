cmake_minimum_required(VERSION 3.5)
project(vision_package)

# Require ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)


#Include directories inside vision_package/include/*
include_directories(include)

# Declare the executable
add_executable(image_publisher src/image_publisher.cpp)

# Link dependencies
ament_target_dependencies(image_publisher
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  image_transport
  vision_msgs
)

# Declare the executable
add_executable(image_subscriber src/image_subscriber.cpp)

# Link dependencies
ament_target_dependencies(image_subscriber
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  image_transport
  vision_msgs 
)

# Install the executables
install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  vision_package/object_detector.py
  vision_package/video_publisher.py
  vision_package/object_tracker.py
  vision_package/track_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
# ament package setup
ament_package()
