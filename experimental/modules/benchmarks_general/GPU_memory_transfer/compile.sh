#!/bin/bash

/usr/local/cuda/bin/nvcc dataTransfer.cu -o ./dataTransfer.o \
    -Xcompiler -march=native -Xcompiler -fopenmp \
    -I/usr/local/cuda/include \
    -L/usr/local/cuda/lib64 -lcudart\
    -Xcompiler -Wno-error=deprecated-enum-enum-conversion \
    -Xcompiler -Wno-deprecated-enum-enum-conversion \
    -lcublas