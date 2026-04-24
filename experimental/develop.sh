#!/bin/bash
set -e
docker build -f DockerFile --target main -t jetson_benchmark:main .

# Main:
xhost +local:docker

DEVICES=""

[ -e /dev/video0 ] && DEVICES="$DEVICES --device=/dev/video0"
[ -e /dev/video1 ] && DEVICES="$DEVICES --device=/dev/video1"

docker run --rm -it $DEVICES --gpus all --volume ./:/src -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix jetson_benchmark:main 
