cmake_minimum_required(VERSION 3.10)
project(pointOdometry)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)

add_library(benchmark_lib
    util/GPU.cu
    util/pipe.cpp
    util/image.cpp
    util/camera.cpp
    ../util/timer.cpp
)

target_compile_options(benchmark_lib PRIVATE -w)

target_link_libraries(benchmark_lib ${OPENCV_LIBRARIES})

add_executable(FIFO_test
    FIFO/FIFO_datarate.cpp
)
target_link_libraries(FIFO_test benchmark_lib)

add_executable(video_pipe
    video/Video.cpp
)
target_link_libraries(video_pipe benchmark_lib)

add_executable(display_pipe
    display/display_images.cpp
)
target_link_libraries(display_pipe benchmark_lib)

add_executable(camera_pipe
    camera/camera_pipe.cpp
)
target_link_libraries(camera_pipe benchmark_lib)

#cmake_minimum_required(VERSION 3.18)
#project(MyProject LANGUAGES CXX CUDA)


add_executable(GPU_memory_transfer
    GPU_memory_transfer/dataTransfer.cu
)
target_link_libraries(GPU_memory_transfer 
    benchmark_lib
    CUDA::cudart
    CUDA::cublas
)



#add_executable(GPU_test
#    GPU_/pointOdometry.cpp
#)
#add_executable(GPU_test
#    GPU_/pointOdometry.cpp
#)
#add_executable(GPU_test
#    GPU_/pointOdometry.cpp
#)