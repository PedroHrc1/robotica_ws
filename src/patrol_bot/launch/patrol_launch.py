from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('rm_description'),
        'launch',
        'robot_simulation.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        Node(
            package='patrol_bot',
            executable='rotate_node',
            name='rotate_node',
            output='screen'
        ),

        Node(
            package='patrol_bot',
            executable='patrol_node',
            name='patrol_node',
            output='screen'
        ),
    ])