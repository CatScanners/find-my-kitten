---
parent: Hardware setup
---

# Baseboard setup - deviations from tutorial

The following tutorial was used for the baseboard setup, and ethernet link between the Jetson and the baseboard: <https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html>.

The goals were achieved, albeit with slightly different configurations.

- Jetpack 6.2 would not flash, with the output log that follows. 6.1 was used instead.
```
18:34:46 INFO: Drivers for Jetson - target_image: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/Jetson_Linux_R36.4.3_aarch64.tbz2
18:34:46 INFO: Gstreamer - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-gstreamer_36.4.3-20250107174145_arm64.deb
18:34:46 INFO: CUDA Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-cudadebuggingsupport_12.6-34622040.0_arm64.deb
18:34:46 INFO: DLA Compiler - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-dla-compiler_36.4.3-20250107174145_arm64.deb with correct checksum, skip downloading.
18:34:46 INFO: CUDA Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-cudadebuggingsupport_12.6-34622040.0_arm64.deb with correct checksum, skip downloading.
18:34:46 INFO: CuDNN Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/cudnn-local-tegra-repo-ubuntu2204-9.3.0_1.0-1_arm64.deb
18:34:47 INFO: TensorRT Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nv-tensorrt-local-tegra-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb
18:34:48 INFO: TensorRT Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/nv-tensorrt-local-tegra-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb with correct checksum, skip downloading.
18:34:48 INFO: OpenCV Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/OpenCV-4.8.0-1-g6371ee1-aarch64-libs.deb
18:34:48 INFO: CuPVA Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/pva-allow-2_2.0.5_all.deb
18:34:48 INFO: CuPVA Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/pva-sdk-2.5-l4t_2.5.4_arm64.deb with correct checksum, skip downloading.
18:34:48 INFO: CuPVA Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/pva-allow-2_2.0.5_all.deb with correct checksum, skip downloading.
18:34:48 INFO: VPI Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/vpi-lib-3.2.4-aarch64-l4t.deb
18:34:48 INFO: VPI Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/vpi-python3.10-3.2.4-aarch64-l4t.deb
18:34:48 INFO: VPI Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/vpi-python3.10-3.2.4-aarch64-l4t.deb with correct checksum, skip downloading.
18:34:48 INFO: VPI Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/vpi-lib-3.2.4-aarch64-l4t.deb with correct checksum, skip downloading.
18:34:48 INFO: NVIDIA Container Runtime with Docker integration (Beta) - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/libnvidia-container1_1.16.2-1_arm64.deb
18:34:48 INFO: NVIDIA Container Runtime with Docker integration (Beta) - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/libnvidia-container-tools_1.16.2-1_arm64.deb
18:34:48 INFO: NVIDIA Container Runtime with Docker integration (Beta) - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/libnvidia-container-tools_1.16.2-1_arm64.deb with correct checksum, skip downloading.
18:34:48 INFO: NVIDIA Container Runtime with Docker integration (Beta) - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/libnvidia-container1_1.16.2-1_arm64.deb with correct checksum, skip downloading.
18:34:48 INFO: Multimedia API - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-jetson-multimedia-api_36.4.3-20250107174145_arm64.deb
18:34:48 INFO: Drivers for Jetson - target_image: Found file /home/younes/Downloads/nvidia/sdkm_downloads/Jetson_Linux_R36.4.3_aarch64.tbz2 with correct checksum, skip downloading.
18:34:48 INFO: CUDA Toolkit for L4T - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/l4t-cuda-tegra-repo-ubuntu2204-12-6-local_12.6.11-1_arm64.deb
18:34:49 INFO: OpenCV - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/OpenCV-4.8.0-1-g6371ee1-aarch64-licenses.deb
18:34:49 INFO: OpenCV - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/OpenCV-4.8.0-1-g6371ee1-aarch64-dev.deb with correct checksum, skip downloading.
18:34:49 INFO: CUDA Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/l4t-cuda-tegra-repo-ubuntu2204-12-6-local_12.6.11-1_arm64.deb with correct checksum, skip downloading.
18:34:49 INFO: VPI on Target - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/vpi-dev-3.2.4-aarch64-l4t.deb
18:34:49 INFO: OpenCV - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/OpenCV-4.8.0-1-g6371ee1-aarch64-samples-data.deb with correct checksum, skip downloading.
18:34:49 INFO: VPI on Target - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/vpi-dev-3.2.4-aarch64-l4t.deb with correct checksum, skip downloading.
18:34:49 INFO: VPI on Target - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/vpi-python-src-3.2.4-aarch64-l4t.deb with correct checksum, skip downloading.
18:34:49 INFO: Nsight Systems - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nsight-systems-2024.5.4.34-3485573-DVS-1_tegra_igpu_arm64.deb
18:34:49 INFO: VPI on Target - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/vpi-samples-3.2.4-aarch64-l4t.deb with correct checksum, skip downloading.
18:34:49 INFO: Nsight Graphics - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/NVIDIA_Nsight_Graphics_L4T_Public_2024.2.24327_arm64.deb
18:34:49 INFO: CuDNN on Target - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/cudnn-local-tegra-repo-ubuntu2204-9.3.0_1.0-1_arm64.deb with correct checksum, skip downloading.
18:35:14 INFO:
18:35:14 INFO: exec_command: sudo rm -rf '/home/younes/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS'
18:35:18 INFO: command finished successfully
18:35:18 INFO: Drivers for Jetson - target_image: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/Jetson_Linux_R36.4.3_aarch64.tbz2
18:35:18 INFO: File System and OS - target_image: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2
18:35:18 INFO: Gstreamer - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-gstreamer_36.4.3-20250107174145_arm64.deb
18:35:18 INFO: DLA Compiler - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-dla-compiler_36.4.3-20250107174145_arm64.deb
18:35:18 INFO: CUDA Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/l4t-cuda-tegra-repo-ubuntu2204-12-6-local_12.6.11-1_arm64.deb
18:35:18 INFO: CUDA Runtime - target: verifying checksum of /home/younes/Downloads/nvidia/sdkm_downloads/nvidia-l4t-cudadebuggingsupport_12.6-34622040.0_arm64.deb
18:35:18 DEBUG: running command < true >
18:35:18 INFO:
18:35:18 INFO: exec_command: true
18:35:18 ERROR: shell-init: error retrieving current directory: getcwd: cannot access parent directories: No such file or directory
18:35:18 INFO: Docker Environment Setup - target_image: [ Component Install Started ]
18:35:18 INFO: Docker Environment Setup - target_image: current working directory is /home/younes
18:35:18 INFO: Docker Environment Setup - target_image: exec_command [host]:
18:35:18 INFO: Docker Environment Setup - target_image: **********************
18:35:18 INFO: Docker Environment Setup - target_image: echo Checking Docker service
18:35:19 INFO: CuDNN Runtime - target: Found file /home/younes/Downloads/nvidia/sdkm_downloads/cudnn-local-tegra-repo-ubuntu2204-9.3.0_1.0-1_arm64.deb with correct checksum, skip downloading.
18:35:19 ERROR: Jetson Docker Image - target_image: Stopping 'docker.service', but its triggering units are still active:
18:35:19 ERROR: Jetson Docker Image - target_image: docker.socket
18:35:24 INFO: Jetson Docker Image - target_image: [host] [ 13.32 MB released. Disk Avail on Partition /dev/mapper/ubuntu--vg-ubuntu--lv: 153.80 GB ]
18:35:24 SUMMARY: Jetson Docker Image - target_image: Install completed successfully.
18:35:24 DEBUG: running command < true >
18:35:24 INFO:
18:35:24 INFO: Drivers for Jetson - target_image: [host] [ Disk Avail on Partition /dev/mapper/ubuntu--vg-ubuntu--lv: 153.80 GB ]
18:35:24 INFO: Drivers for Jetson - target_image: change working directory to /home/younes/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS
18:35:24 INFO: Drivers for Jetson - target_image: The following package was automatically installed and is no longer required:
18:35:24 INFO: Drivers for Jetson - target_image: python3-netifaces
18:35:24 INFO: Drivers for Jetson - target_image: Use 'sudo apt autoremove' to remove it.
18:35:24 INFO: Drivers for Jetson - target_image: 0 upgraded, 0 newly installed, 0 to remove and 9 not upgraded.
18:35:24 INFO: Drivers for Jetson - target_image: [ Package PreInstall Finished Successfully ]
18:35:24 INFO: Drivers for Jetson - target_image: [ Package Install Started ]
18:35:24 INFO: Drivers for Jetson - target_image: current working directory is /home/younes/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS
18:35:24 INFO: Drivers for Jetson - target_image: exec_command [host]:
18:35:24 INFO: Drivers for Jetson - target_image: **********************
18:35:24 INFO: Drivers for Jetson - target_image: tar -I lbzip2 -xf /home/younes/Downloads/nvidia/sdkm_downloads/Jetson_Linux_R36.4.3_aarch64.tbz2
18:35:24 INFO: Drivers for Jetson - target_image: **********************
18:35:24 INFO: Drivers for Jetson - target_image: exec_command: /home/younes/.nvsdkm/replays/scripts/JetPack_6.2_Linux/NV_L4T_DRIVERS_COMP.sh
18:35:33 INFO: Drivers for Jetson - target_image: [ Package Install Finished Successfully ]
18:35:33 INFO: Drivers for Jetson - target_image: [host] [ 784.28 MB used. Disk Avail on Partition /dev/mapper/ubuntu--vg-ubuntu--lv: 153.03 GB ]
18:35:33 INFO: Drivers for Jetson - target_image: [ NV_L4T_DRIVERS_COMP Install took 9s ]
18:35:33 INFO: Drivers for Jetson - target_image: command finished successfully
18:35:33 SUMMARY: Drivers for Jetson - target_image: Install completed successfully.
18:35:33 DEBUG: running command < true >
18:35:33 INFO:
18:35:33 INFO: File System and OS - target_image: exec_command: /home/younes/.nvsdkm/replays/scripts/JetPack_6.2_Linux/NV_L4T_FILE_SYSTEM_AND_OS_COMP.sh
18:36:07 INFO: File System and OS - target_image: Hit:1 http://fi.archive.ubuntu.com/ubuntu noble InRelease
18:36:07 INFO: File System and OS - target_image: Hit:2 http://fi.archive.ubuntu.com/ubuntu noble-updates InRelease
18:36:07 INFO: File System and OS - target_image: Hit:3 http://fi.archive.ubuntu.com/ubuntu noble-backports InRelease
18:36:07 INFO: File System and OS - target_image: Hit:4 https://download.docker.com/linux/ubuntu noble InRelease
18:36:07 INFO: File System and OS - target_image: Hit:5 http://security.ubuntu.com/ubuntu noble-security InRelease
18:36:08 INFO: File System and OS - target_image: Reading package lists...
18:36:08 INFO: File System and OS - target_image: Reading package lists...
18:36:08 INFO: File System and OS - target_image: Building dependency tree...
18:36:08 INFO: File System and OS - target_image: Reading state information...
18:36:08 ERROR: File System and OS - target_image: E: Unable to locate package netcat-openbsd-openbsd
18:36:08 ERROR: File System and OS - target_image: [exec_command]: /bin/bash -c /home/younes/.nvsdkm/replays/scripts/JetPack_6.2_Linux/NV_L4T_FILE_SYSTEM_AND_OS_COMP.sh; [error]: E: Unable to locate package netcat-openbsd-openbsd
18:36:08 INFO: File System and OS - target_image: [ Package Install Finished with Error ]
18:36:08 INFO: File System and OS - target_image: [host] [ 5.52 GB used. Disk Avail on Partition /dev/mapper/ubuntu--vg-ubuntu--lv: 147.51 GB ]
18:36:08 INFO: File System and OS - target_image: [ NV_L4T_FILE_SYSTEM_AND_OS_COMP Install took 34s ]
18:36:08 ERROR: command error code: 11
18:36:08 ERROR: File System and OS - target_image: command terminated with error
18:36:08 SUMMARY: File System and OS - target_image: First Error: Unable to locate debian package
18:36:08 SUMMARY: Multimedia API - target: Depends on failed component
18:36:08 SUMMARY: TensorRT Runtime - target: Depends on failed component
18:36:08 SUMMARY: CUDA Runtime - target: Depends on failed component
18:36:08 SUMMARY: CuDNN Runtime - target: Depends on failed component
18:36:08 SUMMARY: OpenCV Runtime - target: Depends on failed component
18:36:08 SUMMARY: VPI Runtime - target: Depends on failed component
18:36:08 SUMMARY: CuPVA Runtime - target: Depends on failed component
18:36:08 SUMMARY: NVIDIA Container Runtime with Docker integration (Beta) - target: Depends on failed component
18:36:08 SUMMARY: Gstreamer - target: Depends on failed component
18:36:08 SUMMARY: DLA Compiler - target: Depends on failed component
18:36:08 SUMMARY: DateTime Target Setup - target: Depends on failed component
18:36:08 SUMMARY: Flash Jetson Linux - flash: Depends on failed component
```

Unlike what the guide implies, the pixhawk required setting the static ip mentioned. This was done in the MAVLINK console with the following commands:
```
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.41.10.2 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg
echo ROUTER=10.41.10.254 >>/fs/microsd/net.cfg
echo DNS=10.41.10.254 >>/fs/microsd/net.cfg
netman update
```

The netplan configuration file `/etc/netplan/01-netcfg.yaml` did not work for our purposes, which were to both be able to communicate with internet via the RJ45 port, and to the pixhawk via the ethernet bridge.
Communications with the internet did not work originally. This was fixed by adding an additional route and static address, from which the config file finally got the form:
```
network:
  version: 2
  renderer: networkd
  ethernets:
    enP8p1s0:
      dhcp4: yes
      addresses: [10.41.10.1/24, ADDR.1/25]
      routes:
        - to: 0.0.0.0/0
          via: 10.41.10.254
          metric: 100
        - to: 0.0.0.0/0
          via: ADDR.254
          metric: 200
      nameservers:
        addresses: [10.41.10.254, 8.8.8.8, 8.8.4.4]
```
Where ADDR denotes the first parts of the IPV4 address under enP8p1s0 when displaying `ip addr show`, i.e. `222.222.222.55` -> `ADDR = 222.222.222`. Note the subdomain `/25`, which might be different in your case.

Notice that the PX4 parameters may have to be re-configured when updating/reflashing.

Instead of the given `pip install --user -U empy==3.3.4 pyros-genmsg setuptools`, the following were used:
```
sudo apt-get install python3-genmsg
sudo apt-get install python3-setuptools
```
