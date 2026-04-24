#ifndef CAMERA_STREAMER
#define CAMERA_STREAMER
#include <iostream>
#include <fstream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include "image.hpp"

class Streamer {
private:
    const char* dev_name = "/dev/video0";
    const int width  = 1280;
    const int height = 720;
    constexpr static int n_buffers = 2;//should be 2 for low latency
    int next_frame_index = 0;
    int fd;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_format fmt;
    std::vector<IMG> buffers;
    bool stream_is_on = false;
    bool stream_to_gpu_pointer;
public:
    bool init_success = false;

    Streamer(bool stream_to_gpu_pointer);

    ~Streamer() ;

    IMG get_frame() ;


private:
    bool init();
    bool init_steps();
    bool init_device();
    bool init_fmt();
    bool init_request();
    bool init_map();
    bool init_start_stream() ;
    void record_new_image() ;
    v4l2_buffer get_v4l2_buffer(int index) ;
};

#endif 