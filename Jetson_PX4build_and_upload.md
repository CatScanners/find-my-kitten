## How to build PX4 code
- [https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.htm](Guide)
- Follow the guide on the link, except you have to change the ```gcc-multilib``` and ```g++-multilib``` installation lines in ./PX4-Autopilot/Tools/setup/ubuntu.sh to ```gcc-arm-linux-gnueabi```
```g++-arm-linux-gnueabi```.

One can try out if the connection is working by chaning connection code line in MAVSDK-Python/examples/telemetry.py to 
```python
await drone.connect(system_address="serial:///dev/ttyACM0:115200")
```
and by running ```python ~/Documents/MAVSDK-Python/examples/telemetry.py```.\\
Example of the result we should expect: 
<img width="786" alt="image" src="https://github.com/user-attachments/assets/5a60fc0a-68d9-486a-bc75-b8b6e90be06b" />


## Upload it to connected Pixhawk
