[Jetson Orin Nano Developer Kit](<https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/>):
- 1024-core NVIDIA Ampere architecture GPU with 32 Tensor Cores @ 625 MHz
- 6-core Arm® Cortex®-A78AE v8.2 64-bit CPU
- 1.5MB L2 + 4MB L3
- Claimed "Up to 80x performance" over Jetson Nano

Note: it appears our A78AE only supports Neon.

[ARM Cortex A78AE Core Software Optimization Guide](<https://developer.arm.com/documentation/PJDOC-466751330-14665/0600/?lang=en>)

[Neon Programmer's Guide for Armv8-A](<https://developer.arm.com/documentation/102159/0400/Overview>)

[ARM Neon vs. SVE](<https://developer.arm.com/documentation/102131/0100/Overview>), 
[SVE and Neon coding compared](<https://developer.arm.com/-/media/Arm%20Developer%20Community/PDF/Learn%20the%20Architecture/102131_0100_01_SVE_and_Neon_coding_compared.pdf?revision=feaaf72e-a941-461c-bd92-0d960d0f8615>) (PDF), 
[Arm Vector Instructions: SVE and Neon](<https://github.com/NVIDIA/grace-cpu-benchmarking-guide/blob/main/src/developer/vectorization.md>)

# Common licensing:

[Apache 2.0 License](<https://pitt.libguides.com/openlicensing/apache2>):

[ASF 3rd Party License Policy](<https://www.apache.org/legal/resolved.html>)


# [Nvidia Isaac ROS 2.0](<https://www.intermodalics.ai/blog/nvidia-isaac-ros-in-under-5-minutes>)
Isaac ROS 2.0 is a software stack for [ROS2](<https://github.com/ros2>) (Apache 2.0) set of libraries.

It is made up of open source (Apache 2.0) [packages on GitHub](<https://github.com/NVIDIA-ISAAC-ROS>)

https://developer.nvidia.com/embedded/jetpack

https://nvidia-isaac-ros.github.io/performance/index.html

# Object Detection

[Darknet](<https://github.com/pjreddie/darknet>) is a Neural Network framework with You Only Look Once (YOLO) object detection algorithms, available under various open source licenses.

[AI-Powered Video Monitoring: Assessing the NVIDIA Jetson Orin Devices for Edge Computing Applications](<https://ieeexplore.ieee.org/document/10598994/>)
- Performance measurement of on-device YOLOv4 deployed with Nvidia [DeepStream SDK](<https://developer.nvidia.com/deepstream-sdk>) manages 13 FPS average

[3D Flash LiDAR Object Detection and Tracking on Edge Hardware](<https://ieeexplore.ieee.org/document/10670672/>)
- 68 FPS average on Orin Nano with cuNN

# Drone stuff

[Design and Flight Demonstration of a Quadrotor for Urban Mapping and Target Tracking Research](<https://ieeexplore.ieee.org/document/10500131>)
- Following a moving GPS target, scene reconstruction with 5 cameras

[Analysis of Deep Feature Matching Algorithms in UAV Visual Localization](<https://ieeexplore.ieee.org/document/10694214/>)

# CNNs

[Hierarchical Feature Pooling Transformer for Efficient UAV Object Tracking](<https://ieeexplore.ieee.org/document/10247590/>)

[Detecting Violent Behaviour on Edge Using Convolutional Neural Networks](<https://ieeexplore.ieee.org/document/10705272/>)

# SLAM
[High-Speed Stereo Visual SLAM for Low-Powered Computing Devices](<https://ieeexplore.ieee.org/document/10305271/>)
- Jetson-Slam (GPLv3), for original Jetson devices. 60 FPS on Jetson-NX.

