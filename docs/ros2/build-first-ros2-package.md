---
parent: ROS2
---

# Build the first ROS 2 package guide
Prerequisites are to setup ROS 2 based on [jetson-setup.md](https://github.com/CatScanners/find-my-kitten/blob/main/jetson-setup.md)

This guide is based on [ROS2 docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

#### 0. Package structure

Simplest possible cpp package has following file structure:
```
  my_package/
  CMakeLists.txt
  include/my_package/
  package.xml
  src/
```
And python package:
```
  my_package/
  package.xml
  resource/my_package
  setup.cfg
  setup.py
  my_package/

```
Explanations for these required package contents can be found in the [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

#### 1. Create a workspace

Make workspace directory (name can be whatever) and make `src` subdirectory in it:
```
mkdir -p ~/ros2_ws/src
```

All packages in your workspace should be put into the `src` directory.



#### 2. Create a package
First `cd` into `src` folder
```
cd ~/ros2_ws/src
```
Then use following command to make cpp package:
```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```
or command to make python package:
```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
Now there is a new folder in `src` directory called `my_package`.

#### 3. Build packages
`cd` to the root folder
```
cd ~/ros2_ws
```
To build all packages in the workspace use:
```
colcon build
```
To build only `my_package` use:
```
colcon build --packages-select my_package
```
#### 4. Source the setup file
First open a new terminal. Sourcing should always done in new terminal after building to avoid complex issues.

Then `cd` into `ros2_ws` and run the following command:
```
source install/local_setup.bash
```
#### 5. Use the package
To run `my_node`, enter the command:
```
ros2 run my_package my_node
```