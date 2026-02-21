#!/usr/bin/env bash

set -euo pipefail

eprint() {
    echo "$@" >&2
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

REPO_ROOT=$(git rev-parse --show-toplevel)
export ISAAC_ROS_COMMON_CONFIG="$SCRIPT_DIR/.isaac_ros_common-config"

RUN_DEV=src/isaac_ros_common/scripts/run_dev.sh
RUN_DEV_ARGS=()
RUN_DEV_ARGS+=("--isaac_ros_dev_dir" "$REPO_ROOT")
RUN_DEV_ARGS+=("--docker_arg" "--volume /dev/bus/usb:/dev/bus/usb")
RUN_DEV_ARGS+=("--docker_arg" "--device /dev/video0")

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
