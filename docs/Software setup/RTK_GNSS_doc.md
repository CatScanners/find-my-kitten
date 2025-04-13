---
parent: Software setup
---

# RTK GNSS Integration for UAV Systems

This document outlines the hardware and software requirements for integrating RTK GNSS with a UAV system, along with an overview of RTK correction services in Finland.

## Table of Contents
1. [Introduction](#introduction)
2. [Definitions](#definitions)
3. [Hardware Overview](#hardware-overview)
4. [How Corrections Are Received](#how-corrections-are-received)
5. [Power Supply Notes](#power-supply-notes)
6. [Hardware Integration with Drone](#hardware-integration-with-drone)
7. [Available Free RTK Services](#available-free-rtk-services)
    - [National Land Survey of Finland](#national-land-survey-of-finland)
8. [Additional Resources](#additional-resources)
9. [References](#references)

---

## Introduction

This document focuses on the following key topics:
- **Hardware used for RTK GNSS integration.**
- **Steps to connect and integrate the hardware with the drone.**
- **Details of RTK correction services available in Finland.**

---

## Definitions

### **GNSS (Global Navigation Satellite System)**
A global navigation system comprising multiple satellite constellations:
- **GPS (US)**
- **GLONASS (Russia)**
- **Galileo (EU)**
- **BeiDou (China)**

### **RTK (Real-Time Kinematic)**
RTK uses correction data from fixed ground stations to provide precise positioning (up to 1 cm accuracy). It operates by comparing satellite signals received by a **base station** and a **mobile receiver** (e.g., UAV or field device) in real-time.

### **RTK GNSS**
A positioning technique leveraging both GNSS signals and RTK corrections to achieve high-precision positioning. It is particularly effective in open areas with clear satellite visibility. It uses the principle of calculating the distance between a base station and a drone, to establish precise positioning.

To compute the RTK position, both receivers (drone, base station) must have a similar view of the satellites. Because of this, RTCM corrections are effective only within a range of about 35 km from the base station. However, this distance can vary depending on the quality of the antenna and receiver; some sources suggest a range of 25 km, while others claim it can extend up to 50 km. (ardusimple.com)

### **Useful Videos**
- [What is RTK? (YouTube)](https://www.youtube.com/watch?v=C0noxfMWOVY)
- [RTK GNSS Explained (YouTube)](https://www.youtube.com/watch?v=ieearzWTCZw)

---

## Hardware Overview

We will be using the **H-RTK ZED-F9P Rover (RM3100 Compass)**. 

### **Key Features**
- Multi-band RTK capabilities with rapid convergence times.
- Centimeter-level accuracy using base stations or NTRIP services.
- Robust design (IP66 rating) for challenging environments.
- Supports GPS (L1 & L2), GLONASS, Galileo, and BeiDou signals. (We will probably need only GPS).
- Integrated RM3100 compass for accurate orientation.

📄 **More details:**
[H-RTK ZED-F9P Rover](https://holybro.com/collections/standard-h-rtk-series/products/h-rtk-zed-f9p-rover)
[ZED-F9P Integration manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)

---

## How Corrections Are Received

### **RTK Correction Workflow**
1. The ZED-F9P receives RTK corrections via the **NTRIP protocol** (Networked Transport of RTCM via Internet Protocol).
2. Corrections are delivered over the internet, typically using an NTRIP client running on:
   - A ground control station (e.g., QGroundControl).
   - A companion computer.

3. After receiving corrections, the ZED-F9P:
   - Processes satellite signals and applies corrections.
   - Outputs precise position data using standard protocols:
     - **UBX**
     - **NMEA**

4. **Data Flow:**
   - **ZED-F9P → Flight Controller (Pixhawk)**:
     - **UART (GPS Port):** Direct serial connection for positioning data.
     - **DroneCAN:** A more robust CAN bus interface for advanced UAVs.

---

## Power Supply Notes

The ZED-F9P requires a **4.75V–5.25V DC input** for stable operation. 

- **Power Source:** It can be powered via:
  - The flight controller’s GPS or CAN port.
  - An external power supply (if needed for specific setups).

📄 **More technical details:** [ZED-F9P Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)

---

## Hardware Integration with Drone

The following diagram illustrates a typical wiring setup for the H-RTK ZED-F9P module:

![ZED-F9P Connection Diagram](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/ZED-F9P_RTK_Connection_Diagram.jpg?v=1724377596)

### **Steps for Integration (High level overview)**
1. **Mount the ZED-F9P module securely** on the drone.
2. **Connect to Flight Controller:**
   - Use **UART** for GPS data or **CAN Bus** for robust communication.
3. **Configure the NTRIP client** on the ground station or a companion computer.
4. **Test connection and accuracy** using QGroundControl.

---

## Available Free RTK Services

Below is a list of RTK correction services that available in Finland. The priority of those services is as follows: 1. NLS, 2. EUREF, 3. RTK2GO.

### **1. National Land Survey of Finland (NLS)**
- Provides reliable, localized corrections using the NTRIP protocol.
- Best suited for UAV research in Finland.
- **Service Details:**
  - URL: `opencaster.nls.fi`
  - Ports:
    - **2101 (unencrypted)**
    - **2105 (TLS encrypted)** (may not work on all devices)
  - Requires username and password (created during registration).

📄 **More information:** [NLS RTK Service](https://www.maanmittauslaitos.fi/en/finpos/rtk)

**Latest update:**
Currently, the account is made and request for connecting the NLS is sent. The response is not yet received.

### **2. EUREF**
- Europe-wide correction service with established reliability.
- A network of continuously operating GNSS reference stations,
- Data centres providing access to the station data,
- Some base stations from NLS are included here for free use.

**Latest update:**
Currently, the account is made and request for connecting the NLS is sent. The response is not yet received.

### **3. RTK2GO**
- Community-driven RTK service.
- Coverage depends on user-provided base stations (quality varies).

Here is the instructions on how to get corrections from base stations (taken from: <http://rtk2go.com/how-to-connect/>):
Connecting as a Data User…   (users, rovers, tractors)
When connecting to RTK2go as an NTRIP Client no log-on account is required (for the user name just provide a valid email and and the password value is ignored).

Point your NTRIP client to:  rtk2go.com:2101   (port 2101).
Select the valid MountPt name you want and enter it (this string is case sensitive)
For the user account, enter your email.
For the password enter “none”  (the Caster will ignore this value but some software requires it be present).
Please enter a valid email as the user name (i.e. me@myMail.com) then the SNIP Caster (the software the runs RTK2go) will be able to send you limited email reports about what went wrong when your connection attempt does not work. This can be very helpful for quick self debugging uses and is unique to SNIP® NTRIP Casters.  We will not use your email for any other purposes.

If your NTRIP Client software cannot resolve DNS names, point it to:  3.143.243.81:2101

You can connect using either NTRIP Rev1 style or Rev2.
Many low-end NTRIP Clients only support Rev1.

---

## Additional Resources
## WIP (decoder libraries; proper referencing, e.g. Harvard style)

### **Decoder Libraries**
- [pynmea2](https://github.com/Knio/pynmea2) (MIT-license)

---

## References

- [What is GNSS RTK? (gpsgeometer.com)](https://gpsgeometer.com/en/blog/what-is-gnss-rtk-and-how-does-it-work)
- [RTK GNSS Overview (gnss.store)](https://gnss.store/blog/post/what-is-rtk-gnss.html)
- [Centimeter Precision GPS/GNSS - RTK Explained](https://www.ardusimple.com/rtk-explained/)