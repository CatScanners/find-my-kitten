#ifndef GPU_H
#define GPU_H

#include "GPU.cu"

extern void memcopy_GPU_to_CPU(void* from, void* result, int n_bytes);
extern void memcopy_CPU_to_GPU(void* from, void* result, int n_bytes);
extern int divup(int a, int b);
extern void* allocate_GPU(int n_bytes);
extern void deallocate_GPU(void* mem);
extern void afterKernelCallCheckErrors();
extern void synchronize();

#endif
