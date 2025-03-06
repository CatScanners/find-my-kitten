cmake_minimum_required(VERSION 3.5)
project(navigation_package)

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
add_executable(target_deviation src/target_deviation.cpp)

# Link dependencies
ament_target_dependencies(target_deviation
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
  target_deviation
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  navigation_package/deviation_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)
# ament package setup
ament_package()
