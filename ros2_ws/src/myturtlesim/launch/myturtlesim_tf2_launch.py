from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# `DeclareLaunchArgument` is used to define the launch argument
# that can be passed from the above launch file or from the console


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scale_forward',
            default_value='0.5',
            description='Linear velocity scale.'
        ),
        DeclareLaunchArgument(
            'scale_rotation',
            default_value='1.0',
            description='Angular velocity scale.'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='myturtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='myturtlesim',
            executable='turtle2_controller',
            name='turtle2_controller',
            arguments=[LaunchConfiguration('scale_forward') , LaunchConfiguration('scale_rotation')]
        ),
        Node(
            package='myturtlesim',
            executable='turtle_tf2_sensor',
            name='turtle_tf2_sensor',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
        Node(
            package='myturtlesim',
            executable='turtle_tf2_broadcaster',
            name='_turtle1_frame',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='myturtlesim',
            executable='turtle_tf2_broadcaster',
            name='_turtle2_frame',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
    ])
