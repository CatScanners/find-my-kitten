cmake_minimum_required(VERSION 3.10)
project(pointOdometry)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_library(pointOdometry_test_lib
    modules/util/pointOdometry.cpp
    modules/util/timer.cpp
    modules/util/examplePoints.cpp
    modules/util/drone.cpp
    modules/util/Quaternion.cpp
    modules/util/vector3D.cpp
)

add_executable(pointOdometry_test
    modules/pointOdometry/pointOdometry.cpp
)

add_library(pointOdometry_lib
    modules/util/pointOdometry.cpp
    modules/util/examplePoints.cpp
    modules/util/drone.cpp
    modules/util/Quaternion.cpp
    modules/util/vector3D.cpp
)

target_compile_options(pointOdometry_test_lib PRIVATE -w)
target_compile_options(pointOdometry_test PRIVATE -w)
target_compile_options(pointOdometry_lib PRIVATE -w)

target_link_libraries(pointOdometry_test_lib ${OPENCV_LIBRARIES})
target_link_libraries(pointOdometry_test pointOdometry_test_lib)

target_compile_definitions(pointOdometry_test_lib PRIVATE HAS_OPENCV=1)
