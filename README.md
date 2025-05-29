# find-my-kitten

This repository contains the ROS2 packages and the documentation website files for the Find My Kitten, AI! software project.

Documentation website here: https://catscanners.github.io/find-my-kitten/

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
`