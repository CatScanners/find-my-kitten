# Documentation for vision_package

## Introduction

vision_package is a **ROS2** package for to bridge machine vision output to navigation_package. It is meant to be selfcontained piece of the project which in operation only outputs tracked target position to the nodes which handle the drone movement logic. In essence the output is the target bounding box. See node object_tracker.py for more detailed explanation.  

## Nodes and their short explanations

Nodes included in vision_package (where given ros arguments are the default values):

### image_publisher
Publishes regular webcam from /dev/video0 as images in the topic **/image_topic** by default. See the arguments for default values and their types. Do note that this node does work with color when ran with the Arducam global shutter camera that we are using on our drone. For color publishing see arducam_publisher.py 
``` 
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

## arducam_publisher.py
Publishes Adrucam AR02434 global shutter camera with color. Node image_publisher would only publish grayscale images with the cam. The node runs a command to enable BA10 video in V4L2 and shufles the video frames to publish color images. Please see the code of the node for details.
```
ros2 run vision_package arducam_publisher.py --ros-args \
  -p topic_name:="image_topic" \
  -p camera_id:=0 \
  -p res_width:=1920 \
  -p res_width:=1080 \
  -p pub_time:=0.05
```
The input arguments are the same as with image_publisher.
## video_publisher.py
Publishes video or images from local files or links. Online videos are youtube and example of specifying a source can be seen below. This is meant for testing, debugging and other work where pre-existing data needs to be input to the system.   
``` 
ros2 run vision_package video_publisher.py --ros-args \
  -p topic_name:="image_topic" \
  -p input_source:="LINK/PATH" \
  -p pub_time:=0.1
```
topic_name is the ros2 topic name to which the images are published. \
input_source is the source for the data. Can be local video or image. Youtube videos are also supported. 
pub_time is how often frames are published from the input video. 

### image_subscriber
Subscribes to the topic to which image_publisher publishes images. This nodes opens a window in which it displays images it receives. In many ways this a depcrated node as ROS2 has many other options to displaying images such as RVIZ2. Still this is can useful node for when nothing else is available and is kept in the package.    
``` 
ros2 run vision_package image_subscriber --ros-args \
  -p topic_name:="image_topic"
```
topic_name is the topic which this nodes subscribes to and displays the images. 
### object_detector.py
Ultralytics pretrained yolov5 model based object detector node. Listens to /image_topic for input. Outputs yolo bounding box positions as Detection2DArray messages to output topic. Outputs also the dimensions of the image that the detection was ran on.   
``` 
ros2 run vision_package object_detector.py --ros-args \
  -p input_topic_name:="image_topic" \
  -p output_topic_name"="detected_objects_topic" \ 
  -p image_dimension_topic:"image_dimension_topic"
```
input_topic_name is the topic being listened to.
output_topic_name is the topic which this node publishes [Detection2DArray](https://github.com/ros-perception/vision_msgs/blob/ros2/vision_msgs/msg/Detection2DArray.msg) messages to.
### object_tracker.py
Node which uses detections to track the objects with [ByteTrack](https://github.com/ifzhang/ByteTrack) algorithm. The function of this node is persistance. Object detection can drop detection for few frames which easily throws off any follow logic. This is why a tracking algorithm is needed to give more uniform flow of data. This node wants specifically Detection2DArray messages from package [vision_msgs](https://github.com/ros-perception/vision_msgs/). Node uses pre-existing implemenation from [Supervision library](https://supervision.roboflow.com/latest/how_to/track_objects/). object_tracker.py outputs a single object which is the one being tracked.\
\
**!THERE IS NO TARGET SELECTION IMPLEMENTED!**\
This means that currently the tracked object is the first object in the Detection2DArray array. For choosing a specific target that needs to be tracked, one needs to know which type of object needs to be tracked and the detection model specific ID for the object. This is not implemented in the current form of node. 

```
ros2 run vision_package object_tracker.py --ros-args \
  -p input_topic_name:="detected_objects_topic" \
  -p output_topic_name:="tracked_objects_topic"
```
input_topic_name is the name of the topic which this node listens from Detection2DArrays.\
output_topic_name is the name of the topic to which this node publishes the tracked object information to. The format is also Detection2Darray but filtered to the tracked object. 

### track_publisher.py
Node which publishes images with the corresponding detection (bounding box + id) drawn to frame for debugging purposes. Node outputs an image and can be viewed with image_subscriber or RVIZ2 etc. 
```
ros2 run vision_package track_publisher.py --ros-args \ 
  -p input_images:="image_topic" \
  -p input_tracks:="tracked_objects_topic" \
  -p output_topic_name:="tracked_image_topic"
```
input_images is the topic publishing images.\
input_tracks is the topic publishing Detection2DArray messages.\
output_topic_name is the topic name to which the visualization images are published

## Diagram for node interaction in vision_package

[Link to the diagram](https://github.com/CatScanners/find-my-kitten/blob/eeb5590c5453b9f34f5382daa1a27019a8bd45a5/docs/assets/vision-node-level-architecture.pdf)

## Notes and debuggin details
These are problems I've ran across and used considerable time to find the fix.

Image_transport needs to be installed separately eg: 
sudo apt install ros-humble-image-transport

Python scripts need to have exectution rights eg:
chmod +x object_recongizer.py

file header for the py file might be also needed: 
#!/usr/bin/env python3

Use numpy 1.24.1 if ROS2 package "CV_bridge" or others come up with errors about being buildt on numpy<2

Do not use pip packages of opencv pyhton. Apt works best
sudo apt install -y python3-opencv

For yt-dlp eg. video publisher ~/.local/bin needs to be exported to shell. Add the following command to your ~/.bashrc
```
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi
```
Arducam globalshutter camera does not have working exposure. Cannot be set manually nor automatically. 
