---
parent: Simulation & Flight Analysis
title: ROSbags
---

### Recording
ROS2 bags are a recorded 'bag' of data logged from ROS topics, that can later be played back or
analyzed. Refer to `ros2 bag --help` for help with the CLI.

### Playing back
Playing a bag (`ros2 bag play <bag>`) will publish the record messages to the topics as
they are. This can be used to test nodes that subscribe to messages multiple times with the same
data, without needing to do real flights over and over. Ie. the image recognition node subscribes to
`/argus/left/image_raw`, so a bag containing messages from that topic can be used after the fact to
run the image recognition node again on the same data. Then you could also run another node that
reads messages published by the image detection, etc.

### Parsing
There are Python libraries made for parsing bags, one such example is `rosbag2_py`. LLMs are quite
capable of writing simple bag analysis scripts when needed.
