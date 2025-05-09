---
parent: Development
title: Information about hardware and licensing
---

# Information about hardware, licensing and possible libraries

[Jetson Orin Nano Developer Kit](<https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/>):
- 1024-core NVIDIA Ampere architecture GPU with 32 Tensor Cores @ 625 MHz
- 6-core Arm® Cortex®-A78AE v8.2 64-bit CPU
- 1.5MB L2 + 4MB L3
- Claimed "Up to 80x performance" over Jetson Nano

Note: it appears our A78AE only supports Neon.

[ARM Cortex A78AE Core Software Optimization Guide](<https://developer.arm.com/documentation/PJDOC-466751330-14665/0600/?lang=en>)

[Neon Programmer's Guide for Armv8-A](<https://developer.arm.com/documentation/102159/0400/Overview>)

[ARM Neon vs. SVE](<https://developer.arm.com/documentation/102131/0100/Overview>), 
[SVE and Neon coding compared](<https://developer.arm.com/-/media/Arm%20Developer%20Community/PDF/Learn%20the%20Architecture/102131_0100_01_SVE_and_Neon_coding_compared.pdf?revision=feaaf72e-a941-461c-bd92-0d960d0f8615>) (PDF), 
[Arm Vector Instructions: SVE and Neon](<https://github.com/NVIDIA/grace-cpu-benchmarking-guide/blob/main/src/developer/vectorization.md>)

## Common licensing:

This project is licensed under MIT in order to be as permissive as possible. Note that many Isaac ROS packages are under the compatible Apache 2.0, but no such source code is directly included in this repository.

[MIT License](https://opensource.org/license/mit)

[Apache 2.0 License](<https://pitt.libguides.com/openlicensing/apache2>):

[ASF 3rd Party License Policy](<https://www.apache.org/legal/resolved.html>)

