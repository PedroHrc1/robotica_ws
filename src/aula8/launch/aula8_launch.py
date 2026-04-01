from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    publisher_node = Node(
        package='aula8',
        executable='publisher',
        name='aula8_publisher',
        output='screen',
    )

    subscriber_node = Node(
        package='aula8',
        executable='subscriber',
        name='aula8_subscriber',
        output='screen',
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])