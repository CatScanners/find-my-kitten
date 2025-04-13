---
parent: Software setup
---

# PX4 build and upload
- Note that all of this can be skipped by using QGroundControl for these steps.
- [Guide](<https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html>)
- Follow the guide on the link, except you have to change the ```gcc-multilib``` and ```g++-multilib``` installation lines in ./PX4-Autopilot/Tools/setup/ubuntu.sh to ```gcc-arm-linux-gnueabi```
```g++-arm-linux-gnueabi```.

One can try out if the connection is working by chaning connection code line in MAVSDK-Python/examples/telemetry.py to 
```python
await drone.connect(system_address="serial:///dev/ttyACM0:115200")
```
and by running ```python ~/Documents/MAVSDK-Python/examples/telemetry.py```.\\
Example of the result we should expect: 
<img width="786" alt="image" src="https://github.com/user-attachments/assets/5a60fc0a-68d9-486a-bc75-b8b6e90be06b" />


## Upload it to connected Pixhawk (run in PX4-Autopilot folder)
1. Change configurations (add modules that you have created via examples-selection in the boardconfig window): ```make px4_fmu-v6c_default boardconfig```
2. Build PX4 locally: ```make px4_fmu-v6c_default```
3. Upload PX4 build to Pixhawk: ```make px4_fmu-v6c_default```
4. Open mavlink shell for debugging: ```./Tools/mavlink_shell.py /dev/ttyACM0``` (second part perhaps optional/does not work)

There is already an example app built this way. You can launch it by running ```px4_simple_app``` in mavlink shell.

## Pixhawk system architecture

<img width="740" alt="image" src="https://github.com/user-attachments/assets/d8117235-353b-487f-88eb-b40df67cea8f" />
