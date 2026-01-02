from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myturtlesim',
            executable='scale_publisher',
            name='scale_publisher',
            parameters=[
                {
                    'scale_forward': 0.5,
                    'scale_rotation': 1.0
                }
            ]
        )
    ])
