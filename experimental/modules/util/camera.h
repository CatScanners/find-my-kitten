#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include "image.h"

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

    Streamer(bool stream_to_gpu_pointer) : stream_to_gpu_pointer(stream_to_gpu_pointer){
        init_success = init();
    }

    ~Streamer() {
        // --- Stop streaming ---
        if (stream_is_on){
            if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) perror("Stream off failed");
        }
        
        // --- Cleanup ---
        for (__u32 i = 0; i < n_buffers; i++) {
            munmap(buffers[i].data, buffers[i].size());
        }
        close(fd);
    }

    IMG get_frame() {
        v4l2_buffer buf = get_v4l2_buffer(next_frame_index);

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("Dequeue buffer failed");
        }

        //cout << "Index: "<< buf.index << endl;
        record_new_image();
        return buffers[buf.index];
    }
    

private:

    bool init(){
        if (init_steps()){
            return true;
        }
        return false;
    }

    bool init_steps(){
        if (!init_device())         return false;
        if (!init_fmt())            return false;
        if (!init_request())        return false;
        if (!init_map())            return false;
        if (!init_start_stream())   return false;
        return true;
    }

    bool init_device(){
        fd = open(dev_name, O_RDWR);
        if (fd < 0) {
            perror("Cannot open device");
            return false;
        }
        return true;
    }

    bool init_fmt(){
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width  = width ;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = v4l2_fourcc('R','G','1','0'); // RG10
        fmt.fmt.pix.field = V4L2_FIELD_NONE;


        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("Setting format failed");
            return false;
        }
        return true;
    }
    bool init_request(){
        // --- Request buffers ---
        v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count  = n_buffers;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
//        if (stream_to_gpu_pointer){
//            req.memory = V4L2_MEMORY_DMABUF;
//        }else{
//            req.memory = V4L2_MEMORY_MMAP;
//        }
          
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("Requesting buffer failed");
            return false;
        }
        return true;
    }

    bool init_map(){
        for (__u32 i = 0; i < n_buffers; i++) {
            v4l2_buffer buf = get_v4l2_buffer(i);

            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("Query buffer failed");
                return false;
            }
            char* data = (char*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, fd, buf.m.offset);

            if (data == MAP_FAILED) {
                perror("mmap failed");
                return false;
            }
            buffers.push_back({data,width,height,2});

            if (buffers[i].size() != buf.length) {
                perror("Incorrect image size");
                cout << "Expecting image with total size of bytes = " << buffers[i].size() << endl;
                cout << "Recieved  image with total size of bytes = " << buf.length             << endl;
                return false;
            }
            
            //if (stream_to_gpu_pointer){
            //    //buffers[i].start =
            //}else{
            //    buffers[i].start = (char*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
            //                                    MAP_SHARED, fd, buf.m.offset);
            //}
            
            if (i == 0){
                if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                    perror("Queue buffer failed");
                    return false;
                }
            }
        }
        return true;
    }

    
    bool init_start_stream(){
        // --- Start streaming ---
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("Stream on failed");
            return false;
        }
        stream_is_on = true;
        return true;
    }

    void record_new_image() {
        next_frame_index = (next_frame_index+1)%n_buffers;
        v4l2_buffer buf = get_v4l2_buffer(next_frame_index); 
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) perror("Requeue buffer failed");

    }

    v4l2_buffer get_v4l2_buffer(int index) {
        v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        //buf.memory = V4L2_MEMORY_DMABUF;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;
        return buf;
    }

    
};
