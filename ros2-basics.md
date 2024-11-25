# ROS 2 - Some basic concepts



## Data distribution service

ROS 2 data distribution service consists of communication pipelines and nodes. 
Nodes have three ways to communicate:
  ##### 1. Publisher/Subscriber
  - In this context pipelines are called *topics* in which *publisher* node publishes information, referred as *message*
  - *Subscriber* nodes of a topic receive published messages.
  ##### 2. Services
  - Node sends *request* to other node (e.g., turn camera 45 degrees).
  - The receiving node follows the request with a *respond* to the requesting node (e.g., with camera image).
  ##### 3. Actions
  - *Client* node sends *goal* to an *action server* node (e.g., move drone to this location)
  - Action server processes the goal and then sends progress updates, referred as *feedback*
    to the client node (e.g., how far the drone is from the location)
  - When the goal is reached the action server sends a *result* to the client (e.g., picture of the location)

## Node parameters

Nodes can be configured to have parameters. This allows modifying of variables without having to edit code 
and recompiling the project. Parameters can be modified by user or an another node when needed.

## Bag files

Bag files can subscribe to multiple nodes and record the data when it comes in.
This data can be then played back and is published to the corresponding topics.
This allows simulating data as it would be experienced by the drone.

## Packages

ROS code is organized as packages. Package contains all the code for a particular functionality. 
These packages are easily distributable and there are many of them available distributed by other developers.