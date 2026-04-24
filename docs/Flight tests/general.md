---
parent: Flight tests
title: General
---

Flying the drone generally needs two people. One person is the operator, and the other handles stuff
on the drone via SSH.

## Wi-Fi Connection
Someone sets up a hotspot on their phone, and at least the drone and SSH guy connect
to it. You can then SSH onto the drone via its local IP. It helps if the phone the hotspot is on
shows the local IPs of connected devices, otherwise you may need to find it via `nmap` or something.

The Jetson must have connected to and remember the hotspot from before, since unless you have a
screen + KBM to interface with the drone, you can't connect it to anything for the first time during
a flight session. If there are other networks that the drone remembers available where it is, it
might not connect to the hotspot. Placing the hotspot right next to the drone as it boots can help.

The Pixhawk on the drone boots up quickly, but the Jetson (the device you connect to) can take a
couple minutes to boot and connect to the hotspot.

All network traffic then goes via the hotspot phone, so if either the drone or the SSH laptop gets
far away from the phone, the connection will get worse and might be cut off. If flying far away from
starting position, it might be good to somewhat follow the drone with the hotspot and laptop.

## Operator
The operator handles the remote control, and overall flight management. Responsibilities include:
- Controlling the drone with the RC transmitter or being ready to take over autonomous flight at any moment
- Announcing actions and ensuring safety
- Managing mission uploads and failsafes
- Connecting to drone via QGroundControl and getting the GPS fix

## SSH Guy
The SSH person connects to the drone via SSH to start ROS2 nodes, launch
files, and monitor system status with QGroundControl. Responsibilities include:
- Starting the necessary launch files for vision, navigation, etc.
- Monitoring ROS2 topics and node status
- Handling any software issues during flight
- Recording data
- Monitoring QGroundControl for battery level
- Ensure PX4 parameters are right
    - e.g. check that `EKF2_EV_CTRL` is what you expect, `SDLOG_MODE` is reasonable, etc.
    - Can do this via QGroundControl (from operator's laptop), or `mavproxy.py` via SSH
    - Either person could do this, this is just what we did

### mavproxy.py
You can use mavproxy to interface with the flight controller (Pixhawk) from the Jetson. Mavproxy
will spit out a lot of garbage text, which makes reading and writing a bit harder, but the commands
all work.
```bash
# Opens the mavproxy console
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600

# Run help to see available commands
> help

# Help for specific command
> help param

# Shows all params
> param show

# Show specific param
> param show <parameter>

# Set parameter
> param set <parameter> <value>
```

### How to start things via SSH
You'd first enter the development container, make sure everything is built (`colcon build`) if there
have been code changes, and remember to source the `install/setup.bash` file after building.

You're best off having only 1 SSH session open, and using [tmux](https://github.com/tmux/tmux/wiki)
(learn the controls for it too!) inside the container to start multiple things at once.

The following assumes you're in `ros2_ws` with the install script sourced.
#### VSLAM
```bash
# Starting VSLAM + stereo camera
ros2 launch visual_navigation stereo_vslam.launch.py
## In another pane, start the transformation node
ros2 run visual_navigation vslam_message_transform
```

#### Downwards camera
```bash
ros2 launch argus-kitten isaac_ros_argus_camera_mono.launch.py
```

#### YOLO
```bash
# Requires a camera stream, e.g. the downwards camera from above
ros2 run vision_package object_detector.py --ros-args -p yolo_file:=/path/to/yolo/model
```

#### Point odometry
```bash
# Requires YOLO
ros2 run visual_navigation point_odometry
```

#### Recording bags
There is a script `record-bag.sh` in the `drone` directory that can be used to record ROS2 bags
easier. It takes as argument a file with a newline separated list of topics to record (make sure
there are no empty lines!). When you've recorded what you wanted, just CTRL+C the recording script.

```bash
# First cd to <repo root>/drone
./record-bag.sh <topics file>
```
If the topics you record have a lot of data per second (like recording high resolution video), the
bag will start dropping messages after a while as it is unable to keep up with writing the data to
disk. This can be somewhat mitigated by increasing the cache size the command uses with the
`--max-cache-size <size in bytes>` flag, check `ros2 bag record --help` for more info on it. This
will however only delay the dropping of messages, as at some point the cache will still fill up if
recording for long enough. The compression flags might help as well, but we haven't tested this.

**WARNING**:  
If after stopping the bag recording the Jetson is shut off too quickly, the bag might not
finish writing to disk completely and thus be incomplete and corrupted. Make sure you keep the
Jetson powered on for a while (we've done 1-2 minutes, better safe than sorry) even after stopping
the recording.


## General guidelines

### The Day Before
- Come up with mission plans
- If you need to announce flight, do it atleast the day prior to flying (Our case we had to inform out university if we were flying in the campus area)
- Charge all batteries (LiPos, Controller, Laptop)
- Make sure code works
- Check weather forecast
- Pack everything
- Plan mission ahead of time (remember to check waypoint height and speed you want to start small 1m/s at 5m is a good starting point)

### Before Flight
- Check suitable weather (not too windy, stormy, and enough visibility)
- Check integrity of drone
- Check area is clear (cars, people, animals, other aircraft)
- Check airspace restrictions and announce flight in flyk.com dronekartta
- Check RTK fix works
- Set mission parameters + safety features (geofence + failsafe) on QGroundControl

### During Flight
- Monitor battery level, land if hits 14.0V (3.5V per cell)
- Remember, communication is the key to a safe flight
- Landing the drone manually is safer than automatic landing

### How to Start Everything
- Power up the drone
- Connect telemetry module to laptop and WiFi with phone hotspot
- Start QGroundControl and connect to drone
- SSH into the drone and launch necessary ROS2 packages (e.g., vision, navigation)
- Upload mission to QGroundControl
- Arm and takeoff

### WiFi + Other Connections
- WiFi: Connect to drone's WiFi network for SSH access
- Telemetry: USB connection to QGroundControl for MAVLink
- RTK: Ensure RTK GPS is connected and fix is achieved
- Antennas: Ensure all antennas (telemetry, WiFi) are vertical and clear

### Quirks
- Battery percentage may not be accurate; monitor voltage directly
- Ensure props are securely fastened
- Check for loose cables before flight
- Weather can change quickly; have contingency plans

### Mission Planning
- Plan waypoints with appropriate heights and speeds
- Include safety margins (clearance from obstacles)
- Test mission in simulation if possible
- Announce flight plans as required

### Don't Use VSLAM for Live Flights yet, still under development
- VSLAM is experimental; rely on GPS/RTK for positioning in live flights
- Use VSLAM only in controlled testing environments
- You can still run the VSLAM nodes for analysis later on, BUT you must set
  [EKF2_EV_CTRL](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_EV_CTRL) PX4
  parameter to 0 (AKA EV(=external vision) control unused)

### Tips/Tricks
- Always have a spotter who checks that no randoms get near the drone
- Communicate clearly with team
- Keep emergency landing zones in mind
- Document everything for post-flight analysis
- Follow local laws and guidelines

Remember to not let the LiPo battery run too empty, refer to this guide: https://oscarliang.com/lipo-battery-guide/
