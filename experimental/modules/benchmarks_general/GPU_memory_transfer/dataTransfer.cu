#include <cuda_runtime.h>
#include "../util/GPU.h"
#include "../kernels/kernel.h"
#include "../../util/timer.hpp"

#include <iostream>
#include <signal.h>
volatile sig_atomic_t stop = 0;
void handle_sigint(int sig) {
    stop = 1;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, handle_sigint);
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <width> <height> <pixel bytes>\n";
        return 1;
    }

    int w = std::atoi(argv[1]);
    int h = std::atoi(argv[2]);
    int p = std::atoi(argv[3]);
    int size = w*h*p;

    char* data = new char[size];
    for (int i = 0; i < size; i++) (data)[i] = i;
    void* gpu = allocate_GPU(size);
    BenchMark t;
    int n = 0;
    while (!stop){
        n++;
        memcopy_CPU_to_GPU(data,gpu,size);
        synchronize();
        t.step_Completed("memcopy_CPU_to_GPU");

        memcopy_GPU_to_CPU(gpu,data,size);
        synchronize();
        t.cycle_Completed("memcopy_GPU_to_CPU");
    }
    deallocate_GPU(gpu);
    delete [] data;
    std::cout << "\nTransfering " << size*t.fps()/1000/1000 <<"MB/s CPU to GPU and back" << std::endl;
    return 0;
}
