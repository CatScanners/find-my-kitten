#!/usr/bin/env -S bash -l
set -euo pipefail

sudo chown -R "$USERNAME":"$USERNAME" "/home/$USERNAME"
source ~/.bashrc

cd ~/isaacsim
./post_install.sh
./isaac-sim.selector.sh

"${ISAACSIM_PYTHON:?} -m pip install --editable ~/PegasusSimulator/extensions/pegasus.simulator"
