#include <iostream>
#include <cstdlib>
#include <cuda_runtime.h>
#include "GPU.h"

void check(cudaError_t err, const char* context) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA error: " << context << ": "
            << cudaGetErrorString(err) << std::endl;
        //std::exit(EXIT_FAILURE);
    }
}

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

int divup(int a, int b) {
    return (a + b - 1)/b;
}
void afterKernelCallCheckErrors() {
    CHECK(cudaGetLastError());
}
void synchronize(){
    CHECK(cudaDeviceSynchronize());
}


#include <cublas_v2.h>

void matMul(float* A, float* B, float* C,
            int ac, int ab, int bc) {
    // A is ac × ab
    // B is ab × bc
    // C is ac × bc

    cublasHandle_t handle;
    cublasCreate(&handle);

    const float alpha = 1.0f;
    const float beta  = 0.0f;

    // cuBLAS uses column-major order.
    // To compute C = A * B in row-major, we compute:
    // Cᵀ = Bᵀ * Aᵀ in column-major.
    cublasSgemm(
        handle,
        CUBLAS_OP_N, CUBLAS_OP_N,
        bc,          // number of columns of Cᵀ (i.e., rows of C)
        ac,          // number of rows of Cᵀ (i.e., columns of C)
        ab,          // shared dimension
        &alpha,
        B, bc,       // Bᵀ treated as column-major B
        A, ab,       // Aᵀ treated as column-major A
        &beta,
        C, bc        // output Cᵀ
    );

    cublasDestroy(handle);
}

