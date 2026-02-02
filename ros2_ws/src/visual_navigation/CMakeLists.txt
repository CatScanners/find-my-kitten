cmake_minimum_required(VERSION 3.8)
project(visual_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


add_executable(stereo_node
  src/stereo_publisher.cpp
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS stereo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(stereo_node
  rclcpp
  sensor_msgs
  camera_info_manager
  stereo_msgs
  depthai
  depthai_bridge
)

ament_package()
