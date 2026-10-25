"""Launch Xbox teleoperation with optional, deliberately separate nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_mode = LaunchConfiguration('input_mode')
    start_policy = LaunchConfiguration('start_policy')
    start_motor = LaunchConfiguration('start_motor')
    model_path = LaunchConfiguration('model_path')
    motor_id = LaunchConfiguration('motor_id')
    can_interface = LaunchConfiguration('can_interface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_mode', default_value='controller',
            description='gamepad_node input mode: controller or keyboard'),
        DeclareLaunchArgument(
            'start_policy', default_value='false',
            description='Start policy inference; requires /policy/observation publisher'),
        DeclareLaunchArgument(
            'model_path', default_value='policies/policy_humanoid.onnx',
            description='Path to ONNX model used when start_policy:=true'),
        DeclareLaunchArgument(
            'start_motor', default_value='false',
            description='Start the RS06 node (enables its motor during startup)'),
        DeclareLaunchArgument(
            'motor_id', default_value='-1',
            description='RS06 CAN motor ID (0..127); set explicitly when start_motor:=true'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='SocketCAN interface for the motor node'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('gamepad_node'), 'launch', 'gamepad_node.launch.py',
            ])),
            launch_arguments={'input_mode': input_mode}.items(),
        ),
        Node(
            package='th_policy_node', executable='policy_node', name='policy_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('th_policy_node'), 'config', 'policy_humanoid.yaml',
                ]),
                {'model_path': model_path},
            ],
            condition=IfCondition(start_policy), output='screen'),
        Node(
            package='robstride_motor', executable='robstride_motor_node',
            name='robstride_motor_node',
            parameters=[{
                'motor_id': ParameterValue(motor_id, value_type=int),
                'can_interface': can_interface,
            }],
            condition=IfCondition(start_motor), output='screen'),
    ])
