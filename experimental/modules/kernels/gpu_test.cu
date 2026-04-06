
__device__ static inline signed char pixelOperation(short i){
    //return (signed char)(((int)sqrt((double)(i))) & 0xFF);
    //return (signed char)(((int)sqrtf((float)(i>>2))) & 0xFF);
    return static_cast<signed char>(i>>8);//+(i&(1<<2))>>10);
}

__global__ void testKernel(char* data, int size){
    int x = threadIdx.x+blockDim.x*blockIdx.x;
    int y = threadIdx.y+blockDim.y*blockIdx.y;
    if (x< size && y < size) data[x] = data[y];
}
#include "../util/GPU.h"
void dummyTestKernel(void* data, int size){
    constexpr int threads = 32;
    int griddy = (int)sqrtf64(size)/threads; //(This is just nonsense don't use this as an example.)
    const dim3 grid(griddy,griddy);
    constexpr dim3 thread(threads, threads);
    testKernel<<<grid, thread,0>>>((char*)data,size);

    CHECK(cudaGetLastError());
}
