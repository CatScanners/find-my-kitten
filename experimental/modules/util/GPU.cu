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
    CHECK(cudaMallocHost((void**)&dataGPU, n_bytes));
    return dataGPU;
}

void deallocate_GPU(void* mem){
    CHECK(cudaFreeHost(mem));
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
void afterKernelCallCheckErrors() {
    CHECK(cudaGetLastError());
}
void synchronize(){
    CHECK(cudaDeviceSynchronize());
}
