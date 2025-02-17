## This file contains a short tutorial on how to inject RTK corrections to the messaging from QGroundControl via MavProxy's builtin NTRIP client.

First, MavProxy must be installed: https://ardupilot.org/mavproxy/. Note that the software is under the GPL liscence. 
MavProxy offers both the ability to route mavLink messages between different endpoints, and a built-in NTRIP client (to fetch RTK corrections from a remote station). This makes it convenient for our use case,
in which we need to "inject" RTK corrections to QGrounControl's messages.

Before starting QGroundControl, plug the telemetry radio or other physical serial link to the laptop, and power on the pixhawk and GPS receiver (the "rover"). Find out what this connection is named in software (e.g. /dev/tty0 on Linux, or COM3 on Windows).
This can be done in Linux by listing the devices `ls /dev/`, and finding the device that shows up after connecting. 
A similar maneuver can be done in Windows through the device manager, by looking at the "Ports (COM & LPT)" section.

By default, QGroundControl listens to the port localhost:14550. In all, MavLink traffic must be routed from QGroundControl, through MavProxy, to the given serial port. If the port is named `COM4`, for example,
MavProxy should be started with the following command:
```
mavproxy --master=COM4 --out 127.0.0.1:14550
```

Once MavProxy is started, the NTRIP client can be configured by the following commands in the MavProxy console:
```
module load ntrip
ntrip set caster CASTERNAME
ntrip set port PORTNO
ntrip set mountpoint MTPNTNAME
ntrip set mountpoint USERNAME
ntrip set mountpiont PASSWORD
ntrip start
```

Here, the capitalized words denote values specified by the specific NTRIP caster, as well as the user that accessed it. Check the monitoring window for the success of this sequence. QGroundControl should be started only after MavProxy is. Otherwise, it will lay claim to the serial port used for telemetry. 
If all goes well, RTK should show up on the QGroundControl GPS info, provided sufficient visibility of satellites and that the GPS is on.´

The starting command sequence can be automated by appending these commands to the end of the C:\Users\OMISTAJA\AppData\Local\.mavproxy\.mavinit.scr file on Windows, or the /home/username/.mavinit.scr file on Linux. 
See https://ardupilot.org/mavproxy/docs/getting_started/mavinit.html for further details.
