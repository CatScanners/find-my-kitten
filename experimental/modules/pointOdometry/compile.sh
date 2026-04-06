#!/bin/bash

#g++ ./pointOdometry.cpp  -fopenmp -o ./pointOdometry.o $(pkg-config --cflags --libs opencv4) -w -O3
g++ ./pointOdometry.cpp  -fopenmp -o ./pointOdometry.o $(pkg-config --cflags --libs opencv4) -w -O3