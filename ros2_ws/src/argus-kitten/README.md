This is a clone of https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera, with just a patch
that allows us to specify the mode of the IMX-219 camera (ie. the resolution + fps it will stream
video at.) The default is too high resolution, it is expensive to transfer and our main use case of
YOLO will downscale it anyway.
