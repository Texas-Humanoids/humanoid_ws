"""Launch gamepad_node (and joy_node in controller mode)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start joy_node (controller mode only) and gamepad_node."""
    input_mode = LaunchConfiguration('input_mode')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_mode', default_value='controller',
            description='controller or keyboard'),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('gamepad_node'), 'config', 'gamepad_node.yaml']),
            description='Parameter file for gamepad_node'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
            condition=LaunchConfigurationEquals('input_mode', 'controller')),
        Node(
            package='gamepad_node', executable='gamepad_node', name='gamepad_node',
            parameters=[config, {'input_mode': input_mode}],
            output='screen'),
    ])
