#ifndef GPU_H
#define GPU_H

#include <cuda_runtime.h>

void check(cudaError_t err, const char* context);
#define CHECK(x) check(x, #x)

void memcopy_GPU_to_CPU(void* from, void* result, int n_bytes);
void memcopy_CPU_to_GPU(void* from, void* result, int n_bytes);
int divup(int a, int b);
void* allocate_GPU(int n_bytes);
void deallocate_GPU(void* mem);
void afterKernelCallCheckErrors();
void synchronize();
void matMul(float* A, float* B, float* C, int ac, int ab, int bc);

#endif
