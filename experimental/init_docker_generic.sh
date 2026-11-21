#!/bin/bash

set -e

#Use this if you don't have nvidia gpu. note that anything that uses gpu will not work.
#usefull for testing basics on any machine.

docker pull ubuntu:22.04
docker tag  ubuntu:22.04 compiler-base-base-benchmark:latest

docker build -f DockerFile --target main -t jetson_benchmark:main .

docker run --rm -it $DEVICES --volume ./:/src -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix jetson_benchmark:main 
