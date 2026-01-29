# find-my-kitten

This repository contains the ROS2 packages and the documentation website files for the Find My Kitten, AI! software project.

Documentation website here: https://catscanners.github.io/find-my-kitten/

## Very quick start
You have our drone and have [set up our toolchain](https://catscanners.github.io/find-my-kitten/Quick%20Start/Quick%20start.html#:~:text=Setup%20toolchain) on the drone? What should you do to get the drone running?
- Pull this repository and ``cd ros2_ws``
- The run operation can be divided into two parts: **drone pilot part** and **ROS2 packages part**.
  - **Drone pilot part**
    - Refer to [this](https://docs.google.com/document/d/1DUjyzkbAegfWW_M4UNErEH7ssDJYN7t6NvSwmUnFjBE/edit?usp=sharing).
  - **ROS2 packages part**
    - This again can be split to two parts: **vision_package** and **px4_handler**.
      - **vision_package**: get the camera input and enable object detection by referring to [Machine vision guide](https://catscanners.github.io/find-my-kitten/Quick%20Start/Quick%20start.html#:~:text=Machine%20vision%20startup)
      - **px4_handler**: once the drone is flying and offboard-mode is enabled, refer to [Actions startup](https://catscanners.github.io/find-my-kitten/Quick%20Start/Quick%20start.html#:~:text=Actions%20startup)
- Want to make any changes? Make your changes on the packages in ``ros2_ws``, ``colcon build --packages-select <your-package>``, ``source install/setup.bash``.

## Development environment
This project uses UV dependency management for python dependencies. To get started:
* Install UV using: `pip install uv`
* Setup venv using `uv sync` in project root directory
* Activate venv: `source .venv/bin/activate`


## Directory structure

Refer to [architecture](https://catscanners.github.io/find-my-kitten/assets/RealLife-Architecture.png) for the general architecture.
Shortly explained the directory structure: 
| Path                                | Description                                                                      |
| ----------------------------------- | -------------------------------------------------------------------------------- |
| `/`                                 | Root of the project. Contains the main README, license, and configuration files. |
| `/ros2_ws/`                         | Contains our ROS2 packages. Run all ROS-commands here (then build will appear in this directory).                                              
| `/ros2_ws/src/px4_handler/`         | px4_handler-package for motions                                                  |
| `/ros2_ws/src/vision_package/`         | vision_package-package for camera and object detection                                              |
| `/ros2_ws/src/launch`                          | Some ROS2 launch files, have not really used.                              |
| `/simulation`                           | Some files for the simulation, refer to [this](https://catscanners.github.io/find-my-kitten/Simulation%20&%20flight%20analysis/Simulation%20setup.html)                                                    |
| `/docs/`                            | Documentation files.                                                             |
