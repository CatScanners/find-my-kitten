---
parent: Software setup
title: Telemetry radio setup for serial communication
---

# Telemetry radio setup for serial communication

With antennas connected, plug the two transmitter/receivers into your computer's USB ports. Wait for the LEDs to turn a solid green (The model of radio on which this has been tested is the Sik Telemetry Radio V3- 100mW 915Mhz, which is simply plug-and-play).

Find out which ports the devices have been connected to. In windows, these can be found under Device Manager -> Ports (COM and LPT). In Linux, they will be displayed as files under the /dev directory (e.g. '/dev/ttyUSB0').
![image](https://github.com/user-attachments/assets/dbb6de01-dbca-487d-bbdf-4502b635eb0f)

With the following python scripts:
'send.py':
```
import serial
ser = serial.Serial('COM4')
#ser.write(32)
ser.write(b'\x00')
```
'receive.py'
```
import serial
ser = serial.Serial('COM5')
while True:
   print(ser.read(1))
```

Run the 'receive' prograa (`python receive.py`), then run the 'send' script. You should see red LEDs flash on both radios, and console output on the 'receive' program:

![image](https://github.com/user-attachments/assets/a1efc991-6108-419a-9b71-394016b9dbc5)
