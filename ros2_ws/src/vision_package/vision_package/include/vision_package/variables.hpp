#include <string>

// File meant to declare variables for easy modification. 

namespace vision_package {
    using time = std::chrono::duration<double>;        // Time alias 

    const std::string RGB_IMAGE_TOPIC = "image_topic"; // ROS2 topic name
    const std::string RGB_CAMERA_ID   = "/dev/video1"; // Cam id, 0 means /dev/video0
    const int         RGB_RES_WIDTH   = 1920 ;         // Resolution width
    const int         RGB_RES_HEIGHT  = 1080 ;         // Resolution height
    const time        RGB_PUB_TIME    = time(0.03);    // Publishing interval
}
