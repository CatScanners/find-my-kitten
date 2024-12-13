## How to build PX4 code
- [https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.htm](Guide)
- Follow the guide on the link, except you have to change the ```gcc-multilib``` and ```g++-multilib``` installation lines in ./PX4-Autopilot/Tools/setup/ubuntu.sh to ```gcc-arm-linux-gnueabi```
```g++-arm-linux-gnueabi```.

## Upload it to connected Pixhawk
