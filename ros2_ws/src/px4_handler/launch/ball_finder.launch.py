from launch_ros.actions import Node
from launch.actions import TimerAction
from launch import LaunchDescription

def generate_launch_description():
    node_one = Node(
        package='px4_handler',
        executable='ball_finder.py',
        name='maneuver_node',
    )

    node_two = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='px4_handler',
                executable='offboard_control',
                name='offboard_node'
            )
        ]
    )

    return LaunchDescription([
        node_one,
        node_two
    ])

