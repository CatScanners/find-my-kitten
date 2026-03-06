#!/usr/bin/env bash

set -euo pipefail

eprint() {
    echo "$@" >&2
}
die() {
    eprint "ERROR: $@"
    exit 1
}

if getopt --test &>/dev/null; then
    die "GNU getopt required"
fi
SHORT_OPTS="sbcd:"
LONG_OPTS="sim,no-build,no-check,docker:"
PARSED_OPTS=$(getopt --options="$SHORT_OPTS" --longoptions="$LONG_OPTS" --name "$0" -- "$@") || die "getopt failed to parse"
eval set -- "$PARSED_OPTS"

RUN_DEV_ARGS=()
USE_ISAACSIM=false
while true; do
    case "$1" in
        -s|--sim)
            USE_ISAACSIM=true
            shift
            ;;
        -b|--no-build)
            RUN_DEV_ARGS+=("--skip_image_build")
            shift
            ;;
        -c|--no-check)
            RUN_DEV_ARGS+=("--builder_arg" "--skip_registry_check")
            shift
            ;;
        -d|--docker)
            RUN_DEV_ARGS+=("--docker_arg" "$2")
            shift 2
            ;;
        --)
            shift
            break
            ;;
    esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"
RUN_DEV="$SCRIPT_DIR/ros2_ws/src/isaac_ros_common/scripts/run_dev.sh"
IMAGE_KEY="ros2_humble.catscanners"

export ISAAC_ROS_COMMON_CONFIG="$SCRIPT_DIR/devcontainer/.isaac_ros_common-config"

if [[ "$USE_ISAACSIM" == true ]]; then
    eprint "Using image with IsaacSim"
    if [[ ! -d "$HOME/isaacsim" ]]; then
        die "Using IsaacSim, but '$HOME/isaacsim' not found."
    fi

    IMAGE_KEY="$IMAGE_KEY.isaacsim"
    RUN_DEV_ARGS+=("--docker_arg" "--volume $HOME/isaacsim:/home/admin/isaacsim")
fi

RUN_DEV_ARGS+=("--isaac_ros_dev_dir" "$SCRIPT_DIR")
RUN_DEV_ARGS+=("--docker_arg" "--volume /dev/bus/usb:/dev/bus/usb")
RUN_DEV_ARGS+=("--docker_arg" "--device /dev/video0")
RUN_DEV_ARGS+=("--image_key" "$IMAGE_KEY")

if [[ ! -x "$RUN_DEV" ]]; then
    eprint "not found: $RUN_DEV"
    eprint "have you initialized submodules?"
    exit 1
fi

if nvidia-smi -L &> /dev/null; then
    RUN_DEV_ARGS+=("--docker_arg" "--runtime nvidia")
    eprint "NVIDIA GPU detected"
else
    eprint "No NVIDIA GPU detected"
fi


exec "$RUN_DEV" "${RUN_DEV_ARGS[@]}" "$@"
