# Common problems

## Isaac Sim startup errors

If Isaac Sim or `kitten_sim` fails during startup, the most common cause is an incomplete shell environment.

Make sure all of these are sourced in the same terminal session:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash   # or /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash  # some docs may refer to setup.sh
```

If any of these are missing, startup can fail due to missing Python modules, missing ROS environment variables, or ROS packages not being discoverable.

## Waiting for heartbeat spam logs

If you see repeated "Waiting for heartbeat" logs, PX4 most likely did not start.

Common causes:

- `simulation/IsaacSim/standalones/isaac_simple_obstacles.py` points to the wrong PX4 directory. It should point to this repository's `simulation/PX4-Autopilot`.
- `simulation/PX4-Autopilot` has not been built yet.

If PX4 is not built, build it by following the steps in `docs/IsaacSim/Installing.md` (PX4-Autopilot section).

## `isaac_run` command not found

If the shell says `isaac_run: command not found`, your Isaac setup was not loaded in the current terminal.

Typical fix:

- Ensure the `isaac_run` function is present in your `~/.bashrc` (see `docs/IsaacSim/Installing.md`).
- Reload your shell config (or open a new terminal) before running Isaac commands.

## ROS distro setup not found under `/opt/ros`

If launch fails with an error similar to "Neither /opt/ros/jazzy/setup.bash nor /opt/ros/humble/setup.bash found", the required ROS distro is not installed (or not available in this environment/container).

Install/source either Jazzy or Humble and rerun.

## Config file path errors on launch

If launch fails with "Config file not found", the `config:=...` argument is wrong.

Use a config path relative to the `kitten_sim` package share, for example:

```bash
ros2 launch kitten_sim kitten_sim.launch.py config:=configs/ball_finder_yolo_simple.yaml
```

## Behavior falls back to defaults or ignores your YAML

If things behave as if your YAML were empty or only partly applied, two common causes are:

- **Typos in the YAML** — wrong key names or indentation so the launch file does not read the blocks you expect (for example `custom_executables` instead of `custom_scripts`, or a misspelled `isaac` key). Typos are not limited to one section; any wrong key can silently drop that entire feature.
- **The `configs` directory is not installed into the package share** — ROS only sees what `CMakeLists.txt` installs. If you added files under `kitten_sim/configs/` but did not install that directory, `config:=configs/...` may not resolve to your files after `colcon build`. Add an install rule, for example:

```cmake
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/
)
```

(Place it next to the existing `install(DIRECTORY launch/ ...)` in `kitten_sim/CMakeLists.txt`, then rebuild.)

## `isaac.args` YAML type is invalid

If launch reports "`isaac.args` must be a YAML list", make sure it is written as a list, not a plain string.

Correct:

```yaml
isaac:
  args: ["--headless"]
```

## `visual_navigation` build issues outside container

If `colcon build` fails outside the container due to `visual_navigation`, build while skipping that package:

```bash
cd ros2_ws
colcon build --packages-skip visual_navigation
```
