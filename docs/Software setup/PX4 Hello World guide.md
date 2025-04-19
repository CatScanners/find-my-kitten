---
parent: Pixhawk, PX4 and QGroundControl
---

# PX4 Hello World guide

These instructions guide you through the process of building and flashing a custom "Hello World" example on the Pixhawk 4. I was using Ubuntu (version 2404.1.66.0) on a Windows 10 computer.

---

## Prerequisites
1. **Download QGroundControl**: Follow the instructions [here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) to install QGroundControl on your system.
2. **Clone the PX4 Repository**: Open a terminal and enter the following command:
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   ```

---

## Build the PX4 Firmware
   a) **Run Initial Build Command** (for Pixhawk 4):
   ```bash
   make px4_fmu-v5_default
   ```

   b) **Install Required Dependencies**: If the build process gives errors about missing dependencies, use the following commands:
   ```bash
   pip3 install kconfiglib
   sudo apt update
   sudo apt install gcc-arm-none-eabi
   pip3 install empy==3.3.4 pyros-genmsg future lxml
   ```
   After installing the dependencies, retry the build command:
   ```bash
   make px4_fmu-v5_default
   ```

---

## Create the "Hello World" Example
1. **Navigate to the Examples Directory**:
   ```bash
   cd PX4-Autopilot/src/examples/
   ```
   
2. **Create a New Folder for Your Example**:
   ```bash
   mkdir hello_world
   ```
   
3. **Inside `hello_world`, create three files**: `CMakeLists.txt`, `hello_world_main.c`, and `Kconfig`. Add the following content to each file:

   - **CMakeLists.txt**:
     ```cmake
     px4_add_module(
         MODULE examples__hello_world
         MAIN hello_world
         STACK_MAIN 2000
         SRCS
             hello_world_main.c
         DEPENDS
     )
     ```

   - **hello_world_main.c**:
     ```c
     #include <px4_platform_common/log.h>

     __EXPORT int hello_world_main(int argc, char *argv[]);

     int hello_world_main(int argc, char *argv[])
     {
         PX4_INFO("Hello World!");
         for (size_t i = 0; i < 6; ++i) {
             PX4_INFO("hello");
         }
         return OK;
     }
     ```

   - **Kconfig**:
     ```plaintext
     menuconfig EXAMPLES_HELLO_WORLD
         bool "hello_world"
         default n
         ---help---
             Enable support for hello_world
     ```

---

## Configure Board Settings
1. **Run the Configuration Command**:
   ```bash
   make px4_fmu-v5_default boardconfig
   ```
   
2. **Navigate in the GUI Window**:
   - Use the arrow keys and the `Enter` key to navigate.
   - Go to the `examples` directory, find your newly created `hello_world` example, and select it by pressing `Enter` or `Space`. A `[ * ]` or `[ X ]` should appear next to it.
   - Save and exit the GUI.

---

## Build the Firmware
Now, build the firmware with your custom example included:
```bash
make px4_fmu-v5_default
```

---

## Flash the Firmware onto Pixhawk
1. **Open QGroundControl**:
   - Go to **Vehicle Setup** > **Firmware**.

2. **Connect the Pixhawk**:
   - Connect your Pixhawk device to the computer. A "Firmware Setup" window should pop up.

3. **Select the Firmware**:
   - Choose **PX4 Pro v1.15.1** (or similar) and click on **Advanced Settings**.
   - Select **Custom firmware file...** and browse to your built file. It is typically located at:
     ```
     PX4-Autopilot/build/px4_fmu-v5_default/px4_fmu-v5_default.px4
     ```
   
4. **Flash the Firmware**:
   - Click OK and wait for the flash to complete. You should see "Upgrade complete" in yellow text followed by a separator line.
   
5. **Complete Setup**:
   - When prompted, click OK in the "Vehicle components setup" pop-up.
   - Return to the map view by clicking the **Back** button (it looks like a Telegram icon in the top-left corner).
   
---

## Run the "Hello World" Example
1. **Access MAVLink Console**:
   - In QGroundControl, click the Q logo in the top-left corner, select **Analyze Tools**, and then **MAVLink Console**.

2. **Run the Example**:
   - Type `hello_world` (the name of your executable) and press Enter. You should see the "Hello World" and some "hello" outputs printed in the console.
