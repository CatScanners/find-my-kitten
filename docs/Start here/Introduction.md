---
parent: Quick Start
title: Introduction
nav_order: 1
---

# Find my kitten, AI!

## Why?

Drones have the potential to revolutionize search and rescue (S&R) operations by saving lives, gathering critical information, and navigating hazardous environments. However, the challenge lies in determining whether current technologies can enable drones to autonomously perform these tasks in complex, GNSS-free environments.

## What?

Our goal is to develop a drone capable of autonomous navigation in obstacle-laden spaces. It can locate, recognize, identify, and notice moving objects and communicate findings to a control center. With S&R operations in complex, GNSS-free environments as the end goal, the product should iterate toward the goal from simple settings such as free areas and narrow capabilities. For safety and compliance, the system will always include a manual override option. 

The code will be available under Apache 2.0 license which is almost as permissive as MIT and is widely used in open-source projects. This ensures that we are not limited to using only MIT or BSD, but can also apply libraries with Apache 2.0 license. A subgoal of the product is to evaluate and report the state-of-the-art of the relevant technologies, such as ROS2 and PX4. We are planning to use Docker, which is installed in Jetson, to run different packages. The idea is to create a code that is not tied up with a specific hardware. Instead, the product should be potentially deployable or customizable for other drone platforms and be available to anyone who wants to learn and play with autonomous drones. 

## For Whom?

The project outcome, i.e., the product, its demonstration, and its feasibility study, will be used by researchers, investors, or other developers for further inspection and development. Therefore, it should be publicly available (e.g. a GitHub repository) and sufficiently documented. The repository will host detailed instructions for setting up the environment and running the software. Users can report issues or discuss ideas through GitHub’s “Issues” and “Discussions” tabs. 
Additionally, users will be able to test our implementation using Gazebo or AirSim simulators that try to show how our system behaves in different conditions and environments.
