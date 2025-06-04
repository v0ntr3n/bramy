from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bramy',
            executable='publisher',
            name='publisher_node'
        ),
        Node(
            package='bramy',
            executable='subscriber_lambda',
            name='subscriber_node'
        )
    ])
