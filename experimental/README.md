This enviroment is meant for testing theoretical approaches without the overhead of ROS2.
The enviroment can be used to mimic ROS2 structure with minimal overhead.

To get started run `init_docker.sh` to pull docker base image to match your own device.
If your device is does not have Nvidia gpu, pull your own docker base image and rename it to `compiler-base-base-benchmark`.

After pulling the docker image run `develop.sh` to get started.

Inside docker container run util/pointOdometry to build and run point odometry tests.

Runing two scripts like util/video and util/display at the same time using tmux is recomended.
This example will display the video from `testData/DroneTest.mp4` using two different binaeirs with video passed thorugh a FIFO pipe.
`
tmux 
#ctrl+b -> c
util/video
#ctrl+b -> 0
util/display
`


`util/camera` help:

The camera code has been previously been tesed with IMX-217
On jetson it can be configured by runing `sudo python /opt/nvidia/jetson-io/jetson-io.py` and sellecting the following options.

``` 
|                Configure Jetson 24pin CSI Connector                |
|                 Configure for compatible hardware                  |
|                          Camera IMX219-A                           |
|                          Save pin changes                          |
|                Save and reboot to reconfigure pins                 |

``` 
Correct configiration can be checked by runing `ls -l /dev/video*`
it should containing video0.
This also can be used to check it works. `nvgstcapture-1.0`
These also might be helpful to try

``` 
gst-launch-1.0 nvarguscamerasrc ! nv3dsink -e
gst-launch-1.0 nvarguscamerasrc ! nvvidconv ! videoconvert ! nv3dsink -e
``` 

To see camera ioctl instructions use:  (camera required)
``` 
sudo apt install v4l-utils
v4l2-ctl
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
``` 
v4l2-ctl also can be used to record video data
``` 
v4l2-ctl -d /dev/video0 \
--set-fmt-video=width=1280,height=720,pixelformat=RG10 \
--stream-mmap  --stream-count=120 --stream-to=video.raw
``` 


Benchmark help on jetson:

On jetson `nvidia-smi` is not supported see max frequency by  `sudo jetson_clocks --show`

To see current resource usage use:
``` 
sudo pip3 install -U jetson-stats
sudo jtop
``` 


