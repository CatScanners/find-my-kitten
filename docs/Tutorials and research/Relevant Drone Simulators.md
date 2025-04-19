---
parent: Tutorials and Research
title: Relevant Drone Simulators
---

# Relevant Drone Simulators
This is a quick overview of potentially useful simulators for the project. Please, add any other relevant information.

## Colosseum (AirSim) 
[Colosseum](https://github.com/CodexLabsLLC/Colosseum) is an active successor to the popular but discontinued Microsoft's [AirSim](https://github.com/microsoft/AirSim) simulator (Microsoft appears to be trying to commercialize it).

Observations
  * based on Unreal Engine (but can work on Unity to some degree too)
  * it was easy to set up (at least in AirSim days)
  * performant, photorealistic, good physics
  * many sensors supported (multiple cameras, IMU, LIDAR, GPS, etc.)
  * supports many ML use cases
  * weather and illumination can be controlled among other properties
  * quite mature (already years ago)
  * easy-to-use APIs, e.g., in Python and C++
  * support for MavLink and PX4
  * MIT license
  

[![Old AirSim Drone Demo Video](http://i3.ytimg.com/vi/-WfTr1-OBGQ/hqdefault.jpg)](https://youtu.be/-WfTr1-OBGQ)


## Flightmare
UZH's [Fligthmare](https://github.com/uzh-rpg/flightmare) seems to be rather similar to AirSim/Colosseum at least in terms of photorealism, physics , and available sensor suit. Unlike AirSim, it is based on Unity. The project is also MIT-licensed. However, it's development seems to be inactive and without any successors.


[![Flightmare Demo](http://i3.ytimg.com/vi/m9Mx1BCNGFU/hqdefault.jpg)](https://youtu.be/m9Mx1BCNGFU)


## Gazebo
The [Gazebo](https://gazebosim.org/) simulator is also easy-to-setup and work with. The [PX4 guide](https://docs.px4.io/main/en/sim_gazebo_gz/) even explains how to setup Gazebo for PX4 software-in-the-loop testing. A drone model similar to our expected drone is even among the default vehicles. It also comes with a permissive, Apache-2.0, license.

However, compared to the previous simulators, the photorealism and performance are lower.


[![PX4 SITL Gazebo Demo](http://i3.ytimg.com/vi/eRzdGD2vgkU/hqdefault.jpg)](https://youtu.be/eRzdGD2vgkU)

## Other links
- [Air Learning](https://opensynthetics.com/dataset/air-learning/): Open-source simulator and a gym environment for deep reinforcement learning research on resource-constrained aerial robots.
