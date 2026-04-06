#ifndef KERNEL
#define KERNEL

#include "gpu_test.cu"
#include "matMul.cu"

extern void dummyTestKernel(void* data, int size);
extern void matMul(float* A, float* B, float* C, int ac, int ab, int bc);

#endif
