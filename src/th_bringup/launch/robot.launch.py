"""Start the full gamepad -> policy -> motor pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

MOTOR_FIELDS = ('joint', 'motor_id', 'can_interface', 'direction', 'offset', 'lower', 'upper')


def load_motors(path, policy_joints):
    """Read and check motors.yaml against the policy's joint list."""
    with open(path) as f:
        motors = (yaml.safe_load(f) or {}).get('motors') or []
    seen = set()
    for motor in motors:
        missing = [field for field in MOTOR_FIELDS if field not in motor]
        if missing:
            raise RuntimeError(f'{path}: motor entry {motor} is missing {missing}')
        if motor['joint'] not in policy_joints:
            raise RuntimeError(f"{path}: {motor['joint']} is not a policy joint")
        key = (motor['can_interface'], int(motor['motor_id']))
        if motor['joint'] in seen or key in seen:
            raise RuntimeError(f"{path}: duplicate joint or motor ID for {motor['joint']}")
        seen.update({motor['joint'], key})
    return motors


def motor_namespace(joint):
    return '/' + joint.removesuffix('_joint')


def launch_pipeline(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    policy_config = arg('policy_config')
    with open(policy_config) as f:
        profile = yaml.safe_load(f)['policy_node']['ros__parameters']
    motors = load_motors(arg('motors_config'), profile['joint_names'])
    as_bool = {'true': True, 'false': False}
    # ROS cannot infer a type for empty lists, so leave the bridge on its defaults without motors.
    mapping = {
        'joint_names': [m['joint'] for m in motors],
        'motor_namespaces': [motor_namespace(m['joint']) for m in motors],
        'directions': [float(m['direction']) for m in motors],
        'offsets': [float(m['offset']) for m in motors],
        'lower_limits': [float(m['lower']) for m in motors],
        'upper_limits': [float(m['upper']) for m in motors],
    } if motors else {}

    actions = [
        Node(
            package='th_policy_node', executable='policy_node', name='policy_node',
            parameters=[policy_config, {'model_path': arg('model_path')}],
            output='screen'),
        Node(
            package='th_policy_bridge', executable='observation_builder',
            name='observation_builder',
            parameters=[{
                'joint_names': profile['joint_names'],
                'default_joint_positions': profile['default_joint_positions'],
                'action_scale': profile['action_scale'],
                'observation_topic': profile['observation_topic'],
                'joint_target_topic': profile['joint_target_topic'],
                'imu_topic': arg('imu_topic'),
                'require_imu': as_bool[arg('require_imu')],
                'require_all_joints': as_bool[arg('require_all_joints')],
            }],
            output='screen'),
        Node(
            package='th_policy_bridge', executable='joint_motor_bridge',
            name='joint_motor_bridge',
            parameters=[{**mapping, 'joint_target_topic': profile['joint_target_topic']}],
            output='screen'),
    ]
    for motor in motors:
        actions.append(Node(
            package='robstride_motor', executable='robstride_motor_node',
            name='robstride_motor_node', namespace=motor_namespace(motor['joint']),
            parameters=[{
                'motor_id': int(motor['motor_id']),
                'can_interface': motor['can_interface'],
            }],
            output='screen'))
    return actions


def generate_launch_description():
    bringup = FindPackageShare('th_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_mode', default_value='controller', description='controller or keyboard'),
        DeclareLaunchArgument(
            'motors_config', default_value=PathJoinSubstitution([bringup, 'config', 'motors.yaml']),
            description='Joint-to-motor mapping and calibration'),
        DeclareLaunchArgument(
            'policy_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('th_policy_node'), 'config', 'policy_humanoid.yaml']),
            description='Policy profile (joint order, default pose, action scale)'),
        DeclareLaunchArgument(
            'model_path', default_value='policies/policy_humanoid.onnx',
            description='ONNX policy, relative to the current directory or absolute'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data', description='sensor_msgs/Imu source'),
        DeclareLaunchArgument(
            'require_imu', default_value='true',
            description='false assumes upright and still when no IMU data arrives (bench only)'),
        DeclareLaunchArgument(
            'require_all_joints', default_value='true',
            description='false reads joints without motors as the default pose (bench only)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('gamepad_node'), 'launch', 'gamepad_node.launch.py'])),
            launch_arguments={'input_mode': LaunchConfiguration('input_mode')}.items()),
        OpaqueFunction(function=launch_pipeline),
    ])
