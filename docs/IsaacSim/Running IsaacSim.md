# Running Isaac Sim

## Requirements

- Linux is required.
- An NVIDIA GPU is required.
- Isaac Sim runtime must be installed and available on your PATH.

## Run a standalone Isaac Sim app

Use the Isaac helper command with a standalone script:

```bash
isaac_run <standalone_app_script>
```

Example from this repository:

```bash
cd simulation/IsaacSim/standalones
isaac_run isaac_simple_obstacles.py
```

## Run inside the container

You can also run Isaac Sim inside the project container. Start it with:

```bash
./start-isaac-dev.sh -cs
```

This creates/starts a container variant with Isaac Sim installed. Inside that container, the same run commands in this document work (`isaac_run ...`, `ros2 launch ...`, and so on).

When running inside the container, some scripts or configs may need path updates because directory roots can differ from host paths.

## Running `kitten_sim` from `ros2_ws`

The `ros2_ws/src/kitten_sim` package can be run:

- inside the Docker container used for ROS2 packages, or
- outside the container on a Linux host (with dependencies installed).

### Setup

1. Sync Python dependencies:

```bash
uv sync
```

1. Source the virtual environment:

```bash
source .venv/bin/activate
```

1. Source ROS 2 (use whichever distro you have installed):

```bash
source /opt/ros/jazzy/setup.bash
# or
source /opt/ros/humble/setup.bash
```

1. Build the ROS workspace:

```bash
cd ros2_ws
colcon build
```

If running outside the container and you want to skip `visual_navigation`:

```bash
colcon build --packages-skip visual_navigation
```

1. Source the workspace:

```bash
source install/setup.bash
```

### Launch

Run:

```bash
ros2 launch kitten_sim kitten_sim.launch.py config:=configs/ball_finder_yolo_simple.yaml
```

## Configuring the launch YAML

The launch file loads the YAML referenced by `config:=...` and starts Isaac Sim, PX4 handlers, cameras, and optional processing nodes based on that file.

The default config is `ros2_ws/src/kitten_sim/configs/ball_finder_yolo_simple.yaml`.

### Supported config sections

- `isaac`
  - `script`: standalone script name (for example `isaac_simple_obstacles.py`).
  - `px4_dir`: path to PX4 Autopilot directory passed to the standalone app.
  - `args`: extra CLI args passed to `isaac_run` (for example `["--headless"]`).
- `px4`
  - `script`: executable name run via `ros2 run px4_handler <script>`.
  - `delay_seconds`: startup delay before launching that PX4 handler script.
- `offboard`
  - `delay_seconds`: startup delay before running offboard control.
- `cameras`
  - Dictionary of camera aliases to topics.
  - Each entry requires `topic`, used by `vision_package image_subscriber`.
- `object_detector`
  - `enabled`: whether to start object detection.
  - Any other keys are passed as ROS parameters to `vision_package object_detector.py` (for example `input_topic_name`, `pytorch_model_path`).
- `custom_scripts`
  - Optional list of extra `ros2 run` commands.
  - Each item supports:
    - `package`
    - `executable`
    - `parameters` (key/value map passed as `--ros-args -p ...`)
    - `remappings` (list of `[from, to]` mappings)

### Example YAML

```yaml
isaac:
  script: "isaac_simple_obstacles.py"
  px4_dir: "/workspaces/isaac_ros-dev/simulation/PX4-Autopilot"
  args: ["--headless"]

px4:
  script: "ball_finder.py"
  delay_seconds: 70.0

offboard:
  delay_seconds: 45.0

cameras:
  down_facing:
    topic: "/drone1/down_facing_camera/color/image_raw"
  zed_left:
    topic: "/zed_left_gray/image_raw"
  zed_right:
    topic: "/zed_right_gray/image_raw"

object_detector:
  enabled: true
  input_topic_name: "/drone1/down_facing_camera/color/image_raw"
  pytorch_model_path: "yolov5s.pt"
```

## `isaac_simple_obstacles.py`: scene and drone definition

`simulation/IsaacSim/standalones/isaac_simple_obstacles.py` is the standalone Isaac app used by `kitten_sim` in the default config.

This script defines:

- which USD environment is loaded (`simple_obstacles.usd`),
- the multirotor model and PX4 backend settings (`px4_dir`, UDP port, autolaunch),
- ROS 2 backend publication settings,
- the drone's initial pose in the world,
- and graphical sensors (down-facing and stereo cameras), including intrinsics, resolution, update rate, and mounting pose.

Use this file when you want to change simulator scene composition, drone setup, camera placement, or simulated sensor behavior.