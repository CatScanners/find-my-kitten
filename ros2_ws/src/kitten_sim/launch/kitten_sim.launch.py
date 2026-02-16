#!/usr/bin/env python3


import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare



def load_config(config_path: Path):
    if not config_path.exists():
        raise RuntimeError(f"Config file not found: {config_path}")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def find_project_paths(share_path: Path) -> tuple[Path, Path, Path]:
    """Find project root, .venv/activate, ros2_ws, and isaac_sim_standalones from package share."""
    share_path = Path(share_path).resolve()
    current = share_path
    while current != current.parent:
        candidate = current.parent / ".venv" / "bin" / "activate"
        if candidate.exists():
            project_root = current.parent
            ros2_ws = project_root / "ros2_ws"
            isaac_sim_standalones = project_root / "simulation" / "IsaacSim" / "standalones"
            return ros2_ws, isaac_sim_standalones, candidate
        current = current.parent
    raise RuntimeError(f"Could not find .venv/bin/activate starting from {share_path}")


def generate_launch_description():
    package_share = FindPackageShare('kitten_sim')


    config_arg = DeclareLaunchArgument(
        'config',
        default_value='configs/ball_finder_yolo_simple.yaml',
        description='Config filename relative to package share'
    )


    def launch_with_config(context: LaunchContext):
        config_filename = LaunchConfiguration('config').perform(context)
        package_share_path = package_share.perform(context)
        config_path = Path(package_share_path) / config_filename
        config = load_config(config_path)


        # Find project root and venv
        share_path = Path(package_share_path).resolve()
        ros2_ws, isaac_sim_standalones, venv_activate = find_project_paths(share_path)


        def env_prefix(extra=None):
            base = (
                "unset LD_LIBRARY_PATH GTK_PATH GIO_MODULE_DIR && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {venv_activate} && "
                f"cd {ros2_ws} && "
                "source install/setup.bash"
            )
            if extra:
                base += " && " + extra
            return base


        actions = []


        # 1. MicroXRCE Agent
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', env_prefix("MicroXRCEAgent udp4 -p 8888 -v 4")],
            output='screen'
        ))


        # 2. Isaac Sim (replace COMMAND_HERE with what you run manually)
        isaac_command = "isaac_run isaac_simple_obstacles.py --headless"  # your working command
        actions.append(ExecuteProcess(
            cmd=['bash', '-ic', env_prefix(f"cd {isaac_sim_standalones} && {isaac_command}")],
            output='screen'
        ))


        # 3. Offboard control
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', env_prefix("ros2 run px4_handler offboard_control")],
            output='screen'
        ))


        # 4. PX4 handler delayed
        px4_delay = config['px4'].get('delay_seconds', 30.0)
        px4_script = config['px4']['script']
        actions.append(TimerAction(
            period=px4_delay,
            actions=[ExecuteProcess(
                cmd=['bash', '-c', env_prefix(f"ros2 run px4_handler {px4_script}")],
                output='screen'
            )]
        ))


        # 5-N. Cameras
        for cam_name, cam_config in config.get('cameras', {}).items():
            cmd = (
                "ros2 run vision_package image_subscriber "
                f"--ros-args -r __node:={cam_name}_sub "
                f"-r /image_topic:={cam_config['topic']}"
            )
            actions.append(ExecuteProcess(
                cmd=['bash', '-c', env_prefix(cmd)],
                output='screen'
            ))


        # Object detector
        if config.get('object_detector', {}).get('enabled', False):
            params = config['object_detector']
            param_str = " ".join(f"--ros-args -p {k}:={v}" for k, v in params.items())
            cmd = f"ros2 run vision_package object_detector.py {param_str}"
            actions.append(ExecuteProcess(
                cmd=['bash', '-c', env_prefix(cmd)],
                output='screen'
            ))


        # Custom scripts
        for script in config.get('custom_scripts', []):
            pkg = script['package']
            exe = script['executable']
            param_str = " ".join(
                f"--ros-args -p {k}:={v}" for k, v in script.get('parameters', {}).items()
            )
            remap_str = " ".join(
                f"--ros-args -r {r[0]}:={r[1]}" for r in script.get('remappings', [])
            )
            cmd = f"ros2 run {pkg} {exe} {param_str} {remap_str}".strip()
            actions.append(ExecuteProcess(
                cmd=['bash', '-c', env_prefix(cmd)],
                output='screen'
            ))


        return actions


    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=launch_with_config),
    ])




