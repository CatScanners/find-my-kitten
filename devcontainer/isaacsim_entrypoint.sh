#!/usr/bin/env bash
set -euo pipefail

sudo chown -R "$USERNAME":"$USERNAME" "$HOME"
source ~/.bashrc

# If empty, get isaacsim
if [[ ! -f "$ISAACSIM_PATH/initialized" ]]; then
    echo "$ISAACSIM_PATH not initialized, downloading isaacsim" >&2
    cd "$ISAACSIM_PATH"
    ZIP="$ISAACSIM_PATH/sim.zip"
    wget -O "$ZIP" "https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-$(uname -m).zip"
    unzip "$ZIP"
    rm "$ZIP"

    ./post_install.sh
    ./isaac-sim.selector.sh
    "${ISAACSIM_PYTHON:?}" -m pip install --editable ~/PegasusSimulator/extensions/pegasus.simulator

    touch "$ISAACSIM_PATH/initialized"
fi
