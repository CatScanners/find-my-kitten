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
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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

set(EXPERIMENTAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../experimental)

add_library(experimental_util STATIC
  ${EXPERIMENTAL_DIR}/modules/util/pointOdometry.cpp
  ${EXPERIMENTAL_DIR}/modules/util/drone.cpp
  ${EXPERIMENTAL_DIR}/modules/util/Quaternion.cpp
  ${EXPERIMENTAL_DIR}/modules/util/vector3D.cpp
)
target_compile_options(experimental_util PRIVATE -w)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${EXPERIMENTAL_DIR}/modules/util
)


add_executable(stereo_node
  src/stereo_publisher.cpp
)

add_executable(vslam_message_transform
  src/vslam_message_transform.cpp
)

add_executable(point_odometry
  src/point_odometry.cpp
)
target_link_libraries(point_odometry experimental_util)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  stereo_node
  vslam_message_transform
  point_odometry
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

ament_target_dependencies(vslam_message_transform
  rclcpp
  nav_msgs
  px4_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(point_odometry
  rclcpp
  std_msgs
  px4_msgs
  vision_msgs
  tf2
  tf2_ros
)


ament_package()
