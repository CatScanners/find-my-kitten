cmake_minimum_required(VERSION 3.5)
project(px4_handler)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)



add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control 
	sensor_msgs 
	rclcpp 
	px4_msgs 
	std_msgs
	vision_msgs
)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

install(TARGETS
	offboard_control
	DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/ball_finder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()