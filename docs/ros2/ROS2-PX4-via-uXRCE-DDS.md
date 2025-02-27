---
parent: ROS2
---

# PX4 and Jetson communication via uXRCE-DDS guide

Prerequisites are to setup ROS 2 based on [jetson-setup.md](https://github.com/CatScanners/find-my-kitten/blob/main/jetson-setup.md)

Based on PX4 [docs](https://docs.px4.io/main/en/middleware/uxrce_dds.html) 

#### Micro XRCE-DDS Agent Installation within ROS 2 workspace

Create a workspace directory for the agent:

```bash
mkdir -p ~/px4_ros_uxrce_dds_ws/src
```

Clone the source code for the [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent):

```bash
cd ~/px4_ros_uxrce_dds_ws/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```

Compile the workspace:
```bash
colcon build
```
Source the `local_setup.bash`:
```bash
source install/local_setup.bash
```
Start the agent with correct settings for connecting to a uXRCE-DDS client.

#### uORB Topics in ROS 2
When Micro XRCE-DDS is enabled, defined uORB topics coming from PX4 will be published in ROS 2. Similarly, defined ROS 2 topics will be published as uORB topics in PX4.

The PX4 yaml file [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) defines the relationship/pairing between PX4 uORB and ROS 2 messages. 

[List of PX4 uORB topics](https://docs.px4.io/main/en/msg_docs/)

#### uORB Messages in ROS 2

To be able to publish into uORB topics in ROS 2, matching message definitions are needed. Headers for the messages can be found from the [px4_msgs ROS 2 package](https://github.com/PX4/px4_msgs).

#### ROS 2 QoS Settings

Default [QoS settings for ROS 2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) do not work with PX4. So if ROS 2 code needs to subscribe to a uORB topic, it will need to use [compatible]() QoS settings.

PX4 uses the following QoS settings for publishers:

```cpp
uxrQoS_t qos = {
  .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
  .reliability = UXR_RELIABILITY_BEST_EFFORT,
  .history = UXR_HISTORY_KEEP_LAST,
  .depth = 0,
};
```
And the following QoS settings for subscribers:
```cpp
uxrQoS_t qos = {
  .durability = UXR_DURABILITY_VOLATILE,
  .reliability = UXR_RELIABILITY_BEST_EFFORT,
  .history = UXR_HISTORY_KEEP_LAST,
  .depth = queue_depth,
};
```
Default ROS 2 QoS settings for publishers work with uORB topics.


