#ifndef GPU_H
#define GPU_H

extern void process_GPU_RG10toBW8(void* from,void* result,int width,int height,int size); //#include "GPU.cu"
extern void memcopy_GPU_to_CPU(void* from, void* result,int width,int height, int size);
extern void memcopy_CPU_to_GPU(void* from, void* result,int width,int height, int size);

#endif
