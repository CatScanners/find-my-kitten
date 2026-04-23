# Documentation for vision_package

## Introduction

vision_package is a **ROS2** package for to bridge machine vision output to navigation_package. It
is meant to be selfcontained piece of the project which in operation only outputs tracked target
position to the nodes which handle the drone movement logic. The main used node is
`object_detector.py`, which uses YOLO to detect and track objects, and then publishes its results
to a topic.

## Nodes and their short explanations

Nodes included in vision_package (where given ros arguments are the default values):

### object_detector.py
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

## Unused + debugging nodes

### video_publisher.py (debugging)
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

### image_publisher (UNUSED)

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

### arducam_publisher.py (UNUSED)

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
### image_subscriber (UNUSED)

Subscribes to the topic to which image_publisher publishes images. This nodes opens a window in which it displays images it receives. In many ways this a depcrated node as ROS2 has many other options to displaying images such as RVIZ2. Still this is can useful node for when nothing else is available and is kept in the package.    
``` 
ros2 run vision_package image_subscriber --ros-args \
  -p topic_name:="image_topic"
```
topic_name is the topic which this nodes subscribes to and displays the images. 
### object_tracker.py (UNUSED)

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

### track_publisher.py (UNUSED)
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
