---
parent: Development
title: Development container
---

# Introduction
The development container is the supported way to develop this project. You can start it up with the
`start_isaac_dev.sh` script in the root of the repository. Use the `--help` flag for info on its
flags. The images take a while to build and download the first time, after that they will be a lot
faster.

The script is based on the `run_dev.sh` script from v3.2 of
[isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/release-3.2). We had to
fork the repository to add some patches (our fork
[here](https://github.com/CatScanners/isaac_ros_common/tree/catscanners-v3.2), note the
`catscanners-v3.2` branch) to improve the experience. Some changes include:
- Use env var `ISAAC_ROS_COMMONG_CONFIG` to load a configuration file from the specified path
  - Previously the script would only load from home directory, or a .gitignored file. This way we
    can include the configuration as part of our repo
- Not always expecting the user to have an NVIDIA GPU
- Some changes to how the intermediate images get tagged, so our CI can publish them for it to use
  - NVIDIA does not give prebuilt images for x86_64 for their newest patch of v3.2

# Our dev environment
We use the isaac_ros_common's default dev environment as a base, and add our own Python and other
dependencies into it. We use UV for versioning our Python dependencies, and rosdep for dynamic
discovery and install of our ros package dependencies. Our own Dockerfile layers are in
`Dockerfile.catscanners` and `Dockerfile.isaacsim` (for when the `--sim` flag is used).

**NOTE ABOUT PYTHON:**  
Since ROS2 installs its own Python dependencies into the system's global Python, you cannot use a
virtual environment at the same time. This is why the Python dependencies must also be installed
into the global Python inside the container, which we do via `uv pip install --system`. The Jetson
also needs a very specific PyTorch due to its, which is installed on aarch64 systems (bit of a hack, yes).

# Internals
The NVIDIA script is a bit of a mess, and unfortunately the best way to figure out all it does is
read it, but we'll try to explain the gist. Check
[here](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/docker_devenv/index.html#development-environment)
for official documentation on customizing.

### Basic idea
Basically it builds multiple Docker images and layers them on top of eachother, then sets up mounts
and other things via CLI args to Docker and drops you in the container.

### Image layers
The script uses a 'key' based system to decide what images to build. By default it uses the keys
`<arch>.ros2_humble`, e.g., `x86_64.ros2_humble`. This has two keys, the architecture, and
`ros2_humble`. The image matching the architecture key goes first in the layers, followed by
`ros2_humble`. The architecture key is always there, but you can specify the whole suffix.

The script then searches for files with filenames matching `Dockerfile.<key>` (e.g.
`Dockerfile.ros2_humble`), and build them in order of first appearance in the keys to the last. It
searches for these in specific directories, which you can specify custom ones with the
`CONFIG_DOCKER_SEARCH_DIRS` key in the config file (it is a bash array). After everything is built,
there is one final image built up of all the images layered together.
