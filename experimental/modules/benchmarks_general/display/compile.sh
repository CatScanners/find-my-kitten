#!/bin/bash

g++ display_images.cpp -o display.o -I/usr/include/opencv4 -L/usr/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc