from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    argus_pkg = get_package_share_directory('isaac_ros_argus_camera')
    
    # Launch downwards camera
    launch_argus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(argus_pkg, 'launch', 'isaac_ros_argus_camera_mono.launch.py')
        )
    )
    
    return LaunchDescription([
        launch_argus,
        Node(
            name='object_detector',
            package='vision_package',
            executable='object_detector.py',
            # arguments=['--ros-args', '--log-level', 'info']
        ),
    ])