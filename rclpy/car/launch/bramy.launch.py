from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bramy',
            executable='camera_publisher',
            name='publisher_node'
        ),
        Node(
            package='bramy',
            executable='get_control',
            name='get_control_node'
        ),
        Node(
            package='bramy',
            executable='tracking',
            name='subscriber_node'
        ),
        Node(
            package='bramy',
            executable='postProcess',
            name='postProc'
        )
        
    ])
