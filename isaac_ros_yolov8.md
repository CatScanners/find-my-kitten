
Note: ssh with GUI via:
`ssh -Y usr@host`, if using XLaunch (VcXsrv), set `$env:DISPLAY="localhost:0"` in PS prior to SSH

Resources:
- [isaac_ros_object_detection documentation](<https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_yolov8/index.html>)
- [Jetson AI Lab Tutorial - Ultralytics YOLOv8](https://www.jetson-ai-lab.com/tutorial_ultralytics.html#__tabbed_2_1)

## Pre-setup in local environment

Go to the ISAAC ROS workspace and set version to release-3.2

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && git pull
git checkout release-3.2
```

Installing libraries (outside container)

```
sudo apt-get install -y curl jq tar
```

Bash script to download asset from NGC

```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_yolov8"
NGC_RESOURCE="isaac_ros_yolov8_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```

Fetch YOLO model of choice (example with yolov8s, see model comparison [here](<https://www.seeedstudio.com/blog/2023/03/30/yolov8-performance-benchmarks-on-nvidia-jetson-devices/>)):
```
cd Downloads && \
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt
```

Install ultralytics and onnx pip packages to export

```
pip3 install ultralytics
pip3 install onnx
```

Invoke the python interpreter

```
python3
```

Export the YOLO model as onnx

```python3
from ultralytics import YOLO
model = YOLO('yolov8s.pt')
model.export(format='onnx')
quit()
```

Copy the onnx model inside the workspace

```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
cp yolov8s.onnx ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
```

ZED-camera specific: clone the zed-ros2-wrapper

```
cd ${ISAAC_ROS_WS}/src && \
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper
```


## In the docker workspace/container

To start, launch the container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

If it fails to launch with `docker: Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: could not apply required modification to OCI specification: error modifying OCI spec: failed to inject CDI devices: failed to inject devices: failed to stat CDI host device "/dev/fb0": no such file or directory: unknown`, try the following:

```
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
```

```
sudo apt-get update && sudo apt update && rosdep update
```

### ZED setup

[Nvidia docs](<https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/zed_setup.html>)

Install the Zed SDK

```
sudo chmod +x ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-aarch64.sh && \
${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-aarch64.sh

```

Install ZED wrapper dependencies

```
cd ${ISAAC_ROS_WS} && \
rosdep install --from-paths src/zed-ros2-wrapper --ignore-src -r -y && \
colcon build --symlink-install --packages-up-to zed_wrapper
```

Try launching the ZED Explorer GUI

```
/usr/local/zed/tools/ZED_Explorer
```


### Yolov8 setup

General dependencies
```
sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt
```

ZED-camera example specific dependencies
```
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-stereo-image-proc ros-humble-isaac-ros-zed
```

Install packages, build the workspace & source it to `.bashrc` (for other terminals in the container)
```
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

### Running the YOLOv8 example

Launch the isaac_ros_examples node:
```
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
launch_fragments:=zed_mono_rect,yolov8 \
model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan \
interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_yolov8/zed2_quickstart_interface_specs.json
```

Enter the docker container in a second terminal and run visualization

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

```
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```

Enter the docker container in a third terminal and run GUI

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

```
ros2 run rqt_image_view rqt_image_view /yolov8_processed_image
```


**WIP** node (needs images redirect from camera):

```
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize.launch.py model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan input_binding_names:=['images'] output_binding_names:=['output0'] network_image_width:=640 network_image_height:=640 force_engine_update:=False image_mean:=[0.0,0.0,0.0] image_stddev:=[1.0,1.0,1.0] input_image_width:=640 input_image_height:=640 confidence_threshold:=0.25 nms_threshold:=0.45 interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_yolov8/zed2_quickstart_interface_specs.json`
```