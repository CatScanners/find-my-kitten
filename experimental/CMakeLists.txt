cmake_minimum_required(VERSION 3.10)
project(pointOdometry)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_executable(pointOdometry
    modules/pointOdometry/pointOdometry.cpp
    modules/util/pointOdometry.cpp
    modules/util/examplePoints.cpp
    modules/util/drone.cpp
    modules/util/Quaternion.cpp
    modules/util/vector3D.cpp
)

target_compile_options(pointOdometry PRIVATE -w)

target_link_libraries(pointOdometry ${OPENCV_LIBRARIES})