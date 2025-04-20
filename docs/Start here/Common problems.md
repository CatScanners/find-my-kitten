---
parent: Quick Start
title: Common problems
---

# Common problems

### PX4-Autopilot/Tools/ubuntu.sh script is not working in Jetson Nano when setting up PX4 environment
- You have to change the ```gcc-multilib``` and ```g++-multilib``` installation lines in ./PX4-Autopilot/Tools/setup/ubuntu.sh to ```gcc-arm-linux-gnueabi``` and ```g++-arm-linux-gnueabi``` respectively.

### I am not able to install QGroundControl, can I skip installing it?
Yes, you can skip installing with some additional overhead in the development process. The main problem of not having QGC is 
that one has to run the following (in the same order?) in the MAVLINK-console: 
```bash
mavproxy.py --master=udp:127.0.0.1:14540
# First, arm the vehicle.
arm throttle
# Then, takeoff to 10m.
takeoff 10
# Set offboard mode.
param set COM_FLTMODE2 7 # 7 = Offboard, as mentioned here https://docs.px4.io/main/en/advanced_config/parameter_reference.html#commander
```
One can also do this in the PX4 console or programmatically with the offboard node code with their respective syntax.