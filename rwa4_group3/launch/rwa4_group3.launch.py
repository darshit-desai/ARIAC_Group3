
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():    
    """Launches the nodes in the package.

    Returns:
        LaunchDescription: The launch description of the nodes.
    """
    return LaunchDescription([
        Node(
            package='rwa4_group3',
            executable='competition_demo',
            name='competitor_interface',
            output='screen'
            ),
    ])
