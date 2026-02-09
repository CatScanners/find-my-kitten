#!/usr/bin/env bash

set -euo pipefail

eprint() {
    echo "$@" >&2
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

RUN_DEV=src/isaac_ros_common/scripts/run_dev.sh

if [[ ! -x "$RUN_DEV" ]]; then
    eprint "not found: $RUN_DEV"
    eprint "have you initialized submodules?"
    exit 1
fi

REPO_ROOT=$(git rev-parse --show-toplevel)

export ISAAC_ROS_COMMON_CONFIG="$SCRIPT_DIR/.isaac_ros_common-config"
exec "$RUN_DEV" --isaac_ros_dev_dir "$REPO_ROOT" \
    --docker_arg "--volume /dev/bus/usb:/dev/bus/usb" \
    --docker_arg "--device /dev/video0/dev/video0" \
    "$@"
