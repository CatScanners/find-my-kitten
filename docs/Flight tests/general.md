---
parent: Flight tests
title: General
---

Flying the drone generally needs two people. One person is the operator, and the other handles stuff on the drone via SSH.

### Operator
The operator handles the remote control, and overall flight management. Responsibilities include:
- Controlling the drone with the RC transmitter or being ready to take over autonomous flight at any moment
- Announcing actions and ensuring safety
- Managing mission uploads and failsafes

### SSH Guy
The SSH person connects to the drone via SSH (typically over WiFi) to start ROS2 nodes, launch files, and monitor system status with QGroundControl. Responsibilities include:
- Starting the necessary launch files for vision, navigation, etc.
- Monitoring ROS2 topics and node status
- Handling any software issues during flight
- Recording data if needed
- Monitoring QGroundControl

## The Day Before
- Come up with mission plans
- If you need to announce flight, do it atleast the day prior to flying (Our case we had to inform out university if we were flying in the campus area)
- Charge all batteries (LiPos, Controller, Laptop)
- Make sure code works
- Check weather forecast
- Pack everything
- Plan mission ahead of time (remember to check waypoint height and speed you want to start small 1m/s at 5m is a good starting point)

## Before Flight
- Check suitable weather (not too windy, stormy, and enough visibility)
- Check integrity of drone
- Check area is clear (cars, people, animals, other aircraft)
- Check airspace restrictions and announce flight in flyk.com dronekartta
- Check RTK fix works
- Set mission parameters + safety features (geofence + failsafe) on QGroundControl

## During Flight
- Monitor battery level, land if hits 14.0V (3.5V per cell)
- Remember, communication is the key to a safe flight

## How to Start Everything
- Power up the drone
- Connect telemetry module to laptop and WiFi with phone hotspot
- Start QGroundControl and connect to drone
- SSH into the drone and launch necessary ROS2 packages (e.g., vision, navigation)
- Upload mission to QGroundControl
- Arm and takeoff

## WiFi + Other Connections
- WiFi: Connect to drone's WiFi network for SSH access
- Telemetry: USB connection to QGroundControl for MAVLink
- RTK: Ensure RTK GPS is connected and fix is achieved
- Antennas: Ensure all antennas (telemetry, WiFi) are vertical and clear

## How to Record
- Use ROS2 bag recording for topics (e.g., images, odometry)
- Record ULog from PX4 for flight data
- Ensure sufficient storage on drone and laptop

## Quirks
- Battery percentage may not be accurate; monitor voltage directly
- Ensure props are securely fastened
- Check for loose cables before flight
- Weather can change quickly; have contingency plans

## Mission Planning
- Plan waypoints with appropriate heights and speeds
- Include safety margins (clearance from obstacles)
- Test mission in simulation if possible
- Announce flight plans as required

## Don't Use vSLAM for Live Flights yet, still under development
- vSLAM is experimental; rely on GPS/RTK for positioning in live flights
- Use vSLAM only in controlled testing environments

## Tips/Tricks
- Always have a spotter
- Communicate clearly with team
- Keep emergency landing zones in mind
- Practice manual control regularly
- Document everything for post-flight analysis
- Follow local laws and guidelines

Remember to not let the LiPo battery run too empty, refer to this guide: https://oscarliang.com/lipo-battery-guide/
