docker build --no-cache . -t vision_package

xhost -local:docker

docker run --privileged -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    vision_package

