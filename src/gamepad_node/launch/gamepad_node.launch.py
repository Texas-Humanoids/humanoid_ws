"""Launch gamepad_node (and joy's game_controller_node in controller mode)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start game_controller_node (controller mode only) and gamepad_node."""
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
            # Unlike joy_node, game_controller_node reports the same button and axis
            # order for every controller SDL recognizes, including Xbox over Bluetooth.
            package='joy', executable='game_controller_node', name='game_controller_node',
            parameters=[{'autorepeat_rate': 20.0}],
            condition=LaunchConfigurationEquals('input_mode', 'controller')),
        Node(
            package='gamepad_node', executable='gamepad_node', name='gamepad_node',
            parameters=[config, {'input_mode': input_mode}],
            output='screen'),
    ])
