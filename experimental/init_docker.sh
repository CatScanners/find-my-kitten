#!/bin/bash

is_jetson() {
    local soc
    soc=$(tr -d '\0' < /proc/device-tree/compatible 2>/dev/null)

    if echo "$soc" | grep -qi "nvidia,tegra"; then
        return 0
    else
        return 1
    fi
}

if is_jetson; then
    echo "This device is an NVIDIA Jetson"
    version="nvcr.io/nvidia/l4t-cuda:12.2.12-devel"
    docker pull $version
    docker tag $version compiler-base-base-benchmark:latest
else
    echo "This device is not NVIDIA Jetson"
    echo "Gambling to guess nvidia version"
    version="nvidia/cuda:$(nvidia-smi | grep "CUDA Version" | sed 's/.*CUDA Version: \([0-9.]*\).*/\1/')-devel-ubuntu22.04"
    echo "This version may or may not be the correct docker image for this device:"
    echo $version
    read -p "Do you want to try pulling it? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "please pull docker image compatable with your device and name it as compiler-base-base-benchmark:latest"
        echo "docker pull {yourimage}"
        echo "docker tag  {yourimage} compiler-base-base-benchmark:latest"
        exit 1
    fi
    docker pull $version
    docker tag $version compiler-base-base-benchmark:latest
fi
