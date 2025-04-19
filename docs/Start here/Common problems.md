---
parent: Start here
---

# Common problems

### PX4-Autopilot/Tools/ubuntu.sh script is not working in Jetson Nano when setting up PX4 environment
- You have to change the ```gcc-multilib``` and ```g++-multilib``` installation lines in ./PX4-Autopilot/Tools/setup/ubuntu.sh to ```gcc-arm-linux-gnueabi``` and ```g++-arm-linux-gnueabi``` respectively.