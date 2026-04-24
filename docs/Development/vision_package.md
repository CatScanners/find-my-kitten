---
parent: Development
title: Documentation for vision_package
---
# Introduction

vision_package is a **ROS2** package for to bridge machine vision output to navigation_package. It
is meant to be selfcontained piece of the project which in operation only outputs tracked target
position to the nodes which handle the drone movement logic. The main used node is
`object_detector.py`, which uses YOLO to detect and track objects, and then publishes its results
to a topic.

## object_detector.py
Node that uses a YOLO model (get one from [here](https://github.com/ultralytics/assets/releases), or
make your own `.engine` file for better performance) to detect and track objects from an image
topic, publish those detections as a Detection2DArray message, as well as the dimensions of the
input image.
```bash
ros2 run vision_package object_detector.py --ros-args \
  -p image_topic:="/argus/left/image_raw" \
  -p detections_topic:="/detections" \
  -p dimension_topic:="/dimensions" \
  -p yolo_file:=<NO DEFAULT, MUST GIVE THIS PARAMETER>
```
You must at least pass the `yolo_file` argument for the node to run. `/argus/left/image_raw` is the
topic published to by `argus-kitten` package's `isaac_ros_argus_camera_mono.launch.py` launch file
(the launch file used to start the downwards camera)

# Debugging nodes

## video_publisher.py
Publishes video or images from local files or links. Online videos are youtube and example of specifying a source can be seen below. This is meant for testing, debugging and other work where pre-existing data needs to be input to the system.   
```bash
ros2 run vision_package video_publisher.py --ros-args \
  -p topic_name:="image_topic" \
  -p input_source:="LINK/PATH" \
  -p pub_time:=0.1
```
topic_name is the ros2 topic name to which the images are published. \
input_source is the source for the data. Can be local video or image. Youtube videos are also supported. 
pub_time is how often frames are published from the input video. 

## image_subscriber

Subscribes to the topic to which image_publisher publishes images. This nodes opens a window in which it displays images it receives. In many ways this a depcrated node as ROS2 has many other options to displaying images such as RVIZ2. Still this is can useful node for when nothing else is available and is kept in the package.    
```bash
ros2 run vision_package image_subscriber --ros-args \
  -p topic_name:="image_topic"
```
topic_name is the topic which this nodes subscribes to and displays the images. 

## track_publisher.py
Node which publishes images with the corresponding detection (bounding box + id) drawn to frame for debugging purposes. Node outputs an image and can be viewed with image_subscriber or RVIZ2 etc. 
```bash
ros2 run vision_package track_publisher.py --ros-args \ 
  -p input_images:="image_topic" \
  -p input_tracks:="tracked_objects_topic" \
  -p output_topic_name:="tracked_image_topic"
```
input_images is the topic publishing images.\
input_tracks is the topic publishing Detection2DArray messages.\
output_topic_name is the topic name to which the visualization images are published

# UNUSED nodes
For publishing our downwards camera (IMX-219), we instead use the
`isaac_ros_argus_camera_mono.launch.py` from `argus-kitten` package.

## image_publisher (UNUSED)

Publishes regular webcam from /dev/video0 as images in the topic **/image_topic** by default. See the arguments for default values and their types. Do note that this node does work with color when ran with the Arducam global shutter camera that we are using on our drone. For color publishing see arducam_publisher.py 
```bash
ros2 run vision_package image_publisher --ros-args \
  -p topic_name:="image_topic" \
  -p camera_id:=0 \
  -p res_width:=1920 \
  -p res_width:=1080 \
  -p pub_time:=0.05
```
topic_name is the identifying name of the ros2 topic. It takes a string as an input. This needs to be same as any listening node. \
camera_id is the /dev/video[X] number of a camera. For an example in our setup the global shutter camera is /dev/video0 eg camera_id=0 \
res_width and res_height are the dimensions of the published images. OpenCV knows how to downscale but upscaling can lead to unknown behaviour. \ 
pub_time is the publishing interval of images in seconds. Default is to publish once every 0.05 seconds. Can greatly affect performance and should be experienced to fit the hardware. 

## arducam_publisher.py (UNUSED)

Publishes Adrucam AR02434 global shutter camera with color. Node image_publisher would only publish grayscale images with the cam. The node runs a command to enable BA10 video in V4L2 and shufles the video frames to publish color images. Please see the code of the node for details.
```bash
ros2 run vision_package arducam_publisher.py --ros-args \
  -p topic_name:="image_topic" \
  -p camera_id:=0 \
  -p res_width:=1920 \
  -p res_width:=1080 \
  -p pub_time:=0.05
```
The input arguments are the same as with image_publisher.
