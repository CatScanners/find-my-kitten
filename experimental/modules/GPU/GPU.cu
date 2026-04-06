#include <iostream>
#include <cstdlib>
#include <cuda_runtime.h>

static inline void check(cudaError_t err, const char* context) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA error: " << context << ": "
            << cudaGetErrorString(err) << std::endl;
        //std::exit(EXIT_FAILURE);
    }
}
#define CHECK(x) check(x, #x)

void* allocate_GPU(int n_bytes){
    void* dataGPU = NULL;
    CHECK(cudaMalloc((void**)&dataGPU, n_bytes));
    return dataGPU;
}

void deallocate_GPU(void* mem){
    CHECK(cudaFree(mem));
}

void memcopy_GPU_to_CPU(void* from, void* to, int n_bytes){
    CHECK(cudaMemcpy(to, from, n_bytes, cudaMemcpyDeviceToHost));
}

void memcopy_CPU_to_GPU(void* from, void* to, int n_bytes){
    CHECK(cudaMemcpy(to, from, n_bytes, cudaMemcpyHostToDevice));
}

static inline int divup(int a, int b) {
    return (a + b - 1)/b;
}


constexpr int short_vec = 4;
constexpr int char_vec = 8;
constexpr int bitshift = 2;

__device__ static inline signed char pixelOperation(short i){
    return static_cast<signed char>(i>>8);
}

__global__ void RG10toBW8(short* from, char* to,int width, int height){
    int x = threadIdx.x+blockDim.x*blockIdx.x;
    int y = threadIdx.y+blockDim.y*blockIdx.y;
    int loc = x+width*y;
    if (x >= width || y >= height) return;
    to[loc] = pixelOperation(from[loc]);
}

#include "timer.h"
void process_GPU_RG10toBW8(char* data, char* result,int width,int height,int size){
    void* from = allocate_GPU(size);
    void* to   = allocate_GPU(height*width);
    memcopy_CPU_to_GPU(data,from,size);
    constexpr int threads = 32;
    constexpr dim3 thread(threads, threads);
    //Timer t;

    const dim3 grid(divup(width, threads), divup(height, threads));
    RG10toBW8<<<grid, thread,0>>>((short*)from,(char*)to,width,height);

    CHECK(cudaGetLastError());
    CHECK(cudaDeviceSynchronize());
    //t.time_stamp();
    deallocate_GPU(from);
    memcopy_GPU_to_CPU(to,result,height*width);
    deallocate_GPU(to);
}
