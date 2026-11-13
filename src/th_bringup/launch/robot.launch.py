"""Start the full gamepad -> policy -> motor pipeline."""

import json

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

MOTOR_FIELDS = (
    'joint', 'motor_id', 'motor_model', 'can_interface', 'direction', 'offset', 'lower', 'upper')


# Seconds a policy target stays valid. The bridge disarms once targets are this old, and each
# MIT motor drops to damping at the same age, so the two never disagree about a stalled policy.
TARGET_TIMEOUT = 0.2

# Observation term sizes; joint-sized terms use None.
OBS_TERMS = {
    'velocity_command': 3, 'base_ang_vel': 3, 'projected_gravity': 3, 'joint_pos_rel': None,
    'joint_vel': None, 'last_action': None, 'gait_phase': 2,
}


SIM_DEFAULTS = {'motor_id': None, 'offset': 0.0}


def load_robot(path, sim=False):
    """Read the motor list plus optional robot-wide settings (imu, control_mode, damping_kd).

    In sim, motor IDs are not used and a missing offset reads as 0.
    """
    with open(path) as f:
        robot = yaml.safe_load(f) or {}
    motors = robot.get('motors') or []
    ids = set()
    for index, motor in enumerate(motors):
        if sim:
            for field, default in SIM_DEFAULTS.items():
                if motor.get(field) is None:
                    motor[field] = index if default is None else default
        missing = [field for field in MOTOR_FIELDS if motor.get(field) is None]
        if missing:
            raise RuntimeError(
                f'{path}: {motor.get("joint", motor)} has no {missing}; fill them in '
                '(see the th_bringup README)')
        key = (motor['can_interface'], int(motor['motor_id']))
        if key in ids:
            raise RuntimeError(f'{path}: motor ID {key[1]} on {key[0]} is listed twice')
        ids.add(key)
    return robot, motors


def load_policy_spec(path):
    """Read the training export (deploy/policy_spec.json) and check it is self-consistent."""
    with open(path) as f:
        spec = json.load(f)
    joints = spec['joint_order']
    n = len(joints)
    for key in ('default_joint_pos', 'kp', 'kd'):
        if len(spec[key]) != n:
            raise RuntimeError(f'{path}: {key} has {len(spec[key])} entries, joint_order has {n}')
    terms = spec['observations']
    size = 0
    for term in terms:
        if term['name'] not in OBS_TERMS:
            raise RuntimeError(f'{path}: unknown observation term {term["name"]}')
        size += OBS_TERMS[term['name']] or n
    if 'observation_size' in spec and spec['observation_size'] != size:
        raise RuntimeError(
            f'{path}: observation terms add up to {size}, spec says {spec["observation_size"]}')
    if any(t['name'] == 'gait_phase' for t in terms) and not spec.get('gait_period'):
        raise RuntimeError(f'{path}: gait_phase observation needs gait_period')
    spec['_observation_size'] = size
    return spec


def motor_namespace(joint):
    return '/' + joint.removesuffix('_joint')


def launch_pipeline(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim = arg('sim') == 'true'
    robot, motors = load_robot(arg('motors_config'), sim)
    spec_path = arg('policy_spec')
    bridge_extra = {}
    gains = {}
    if spec_path:
        spec = load_policy_spec(spec_path)
        profile = {
            'joint_names': spec['joint_order'],
            'default_joint_positions': [float(v) for v in spec['default_joint_pos']],
            'action_scale': float(spec['action_scale']),
            'observation_topic': '/policy/observation',
            'joint_target_topic': '/policy/joint_targets',
        }
        clip = spec.get('action_clip')
        policy_params = [{
            **profile,
            'expected_observation_size': spec['_observation_size'],
            'action_limit_lower': -float(clip) if clip else -10000.0,
            'action_limit_upper': float(clip) if clip else 10000.0,
        }]
        bridge_extra = {
            'observation_terms': [t['name'] for t in spec['observations']],
            'observation_scales': [float(t.get('scale', 1.0)) for t in spec['observations']],
            'gait_period': float(spec.get('gait_period', 0.0)),
            'rate': 1.0 / float(spec['policy_dt']),
        }
        gains = {
            joint: (float(kp), float(kd))
            for joint, kp, kd in zip(spec['joint_order'], spec['kp'], spec['kd'])}
    else:
        policy_config = arg('policy_config')
        with open(policy_config) as f:
            profile = yaml.safe_load(f)['policy_node']['ros__parameters']
        policy_params = [policy_config]
    control_mode = robot.get('control_mode', 'csp')
    if control_mode == 'mit' and not gains:
        raise RuntimeError('control_mode mit needs policy_spec:=... for the kp/kd gains')
    imu = robot.get('imu') or {}
    if 'rotation' in imu:
        bridge_extra['imu_rotation'] = [float(v) for v in imu['rotation']]
    if 'world_frame' in imu:
        bridge_extra['imu_world_frame'] = imu['world_frame']
    if arg('walk_event'):
        bridge_extra['walk_event'] = arg('walk_event')
    if control_mode == 'mit' and motors:
        # The bridge switches each motor between its trained gains (WALK) and optional
        # stiffer stand_kp/stand_kd from the robot yaml (STAND).
        for m in motors:
            if m['joint'] not in gains:
                raise RuntimeError(f'{m["joint"]} is not in the policy spec joint_order')
        bridge_extra['policy_kp'] = [gains[m['joint']][0] for m in motors]
        bridge_extra['policy_kd'] = [gains[m['joint']][1] for m in motors]
        bridge_extra['stand_kp'] = [
            float(m.get('stand_kp', gains[m['joint']][0])) for m in motors]
        bridge_extra['stand_kd'] = [
            float(m.get('stand_kd', gains[m['joint']][1])) for m in motors]
    # ROS cannot infer a type for empty lists, so leave the bridge on its defaults without motors.
    mapping = {
        'motor_joints': [m['joint'] for m in motors],
        'motor_namespaces': [motor_namespace(m['joint']) for m in motors],
        'directions': [float(m['direction']) for m in motors],
        'offsets': [float(m['offset']) for m in motors],
        'lower_limits': [float(m['lower']) for m in motors],
        'upper_limits': [float(m['upper']) for m in motors],
    } if motors else {}

    actions = [
        Node(
            package='th_policy_node', executable='policy_node', name='policy_node',
            parameters=[*policy_params, {'model_path': arg('model_path')}],
            output='screen'),
        Node(
            package='th_policy_bridge', executable='policy_bridge', name='policy_bridge',
            parameters=[{
                'joint_names': profile['joint_names'],
                'default_joint_positions': profile['default_joint_positions'],
                'action_scale': profile['action_scale'],
                'observation_topic': profile['observation_topic'],
                'joint_target_topic': profile['joint_target_topic'],
                'imu_topic': arg('imu_topic'),
                'require_imu': arg('require_imu') == 'true',
                'target_timeout': TARGET_TIMEOUT,
                **bridge_extra,
                **mapping,
            }],
            output='screen'),
    ]
    if sim:
        actions.append(sim_node(arg, robot, motors, control_mode, gains, profile))
        motors = []
    elif imu.get('driver') and arg('require_imu') == 'true':
        actions.append(imu_node(arg, imu))
    for motor in motors:
        params = {
            'motor_id': int(motor['motor_id']),
            'motor_model': motor['motor_model'],
            'can_interface': motor['can_interface'],
            'control_mode': control_mode,
        }
        if control_mode == 'mit':
            if motor['joint'] not in gains:
                raise RuntimeError(f'{motor["joint"]} is not in the policy spec joint_order')
            # Training gains are joint-space; |direction| = 1 so they carry over unchanged.
            params['kp'], params['kd'] = gains[motor['joint']]
            params['damping_kd'] = float(motor.get('damping_kd', robot.get('damping_kd', 1.0)))
            params['feedback_rate'] = float(robot.get('feedback_rate', 100.0))
            params['target_timeout'] = TARGET_TIMEOUT
            if 'torque_limit' in motor:
                params['torque_limit'] = float(motor['torque_limit'])
        actions.append(Node(
            package='robstride_motor', executable='robstride_motor_node',
            name='robstride_motor_node', namespace=motor_namespace(motor['joint']),
            parameters=[params],
            output='screen'))
    if arg('record') == 'true':
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--regex',
                 '/policy/.*|/policy_bridge/.*|/cmd_vel|/gamepad/.*|' +
                 arg('imu_topic') + '|/.*/motor/.*'],
            output='screen'))
    return actions


def imu_node(arg, imu):
    """The IMU driver named by the robot yaml's imu.driver."""
    if imu['driver'] != 'vectornav':
        raise RuntimeError(f'unknown imu.driver {imu["driver"]}; supported: vectornav')
    return Node(
        package='vectornav_imu', executable='vectornav_imu_node', name='vectornav_imu',
        parameters=[{
            'port': imu.get('port', '/dev/ttyUSB0'),
            'baud': int(imu.get('baud', 115200)),
            'rate': float(imu.get('rate', 200.0)),
            'imu_topic': arg('imu_topic'),
        }],
        output='screen')


def sim_node(arg, robot, motors, control_mode, gains, profile):
    """th_sim's MuJoCo robot in place of the robstride motor nodes and the IMU driver."""
    if control_mode != 'mit' or not motors:
        raise RuntimeError('sim:=true emulates MIT-mode motors: needs control_mode mit and motors')
    defaults = dict(zip(profile['joint_names'], profile['default_joint_positions']))
    imu = robot.get('imu') or {}
    return Node(
        package='th_sim', executable='mujoco_sim_node', name='mujoco_sim',
        parameters=[{
            'model_path': arg('sim_model'),
            'joints': [m['joint'] for m in motors],
            'motor_namespaces': [motor_namespace(m['joint']) for m in motors],
            'motor_models': [m['motor_model'] for m in motors],
            'directions': [float(m['direction']) for m in motors],
            'offsets': [float(m['offset']) for m in motors],
            'kp': [gains[m['joint']][0] for m in motors],
            'kd': [gains[m['joint']][1] for m in motors],
            'torque_limits': [float(m.get('torque_limit', 0.0)) for m in motors],
            'damping_kd': float(robot.get('damping_kd', 1.0)),
            'target_timeout': TARGET_TIMEOUT,
            'command_delay': float(arg('sim_command_delay')),
            'feedback_rate': float(robot.get('feedback_rate', 100.0)),
            'initial_joint_positions': [float(defaults[m['joint']]) for m in motors],
            'imu_topic': arg('imu_topic'),
            'imu_rotation': [float(v) for v in imu.get('rotation', [1.0, 0.0, 0.0, 0.0])],
            'imu_world_frame': imu.get('world_frame', 'enu'),
            'gantry_release': arg('sim_gantry_release'),
            'viewer': arg('sim_viewer') == 'true',
            'stream_to': arg('sim_stream_to'),
        }],
        output='screen')


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
            'policy_spec', default_value='',
            description='Training export (policy_spec.json); overrides policy_config when set'),
        DeclareLaunchArgument(
            'walk_event', default_value='',
            description="Two-step arming: START stands, this event (e.g. 'A pressed') walks"),
        DeclareLaunchArgument(
            'record', default_value='false', choices=['true', 'false'],
            description='Record policy, IMU and motor topics with ros2 bag'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data', description='sensor_msgs/Imu source'),
        DeclareLaunchArgument(
            'require_imu', default_value='true', choices=['true', 'false'],
            description='false assumes upright and still, for bench tests without an IMU'),
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='Simulate motors and IMU in MuJoCo (th_sim) instead of the robstride nodes'),
        DeclareLaunchArgument(
            'sim_model',
            default_value='src/th_description/robots/walker_v2/mjcf/scene.xml',
            description='MuJoCo scene for sim:=true, relative to the current directory or absolute'),
        DeclareLaunchArgument(
            'sim_gantry_release', default_value='WALK',
            description='Bridge mode that drops the sim gantry: STAND, WALK, never, or off (no gantry)'),
        DeclareLaunchArgument(
            'sim_command_delay', default_value='0.01',
            description='Seconds before a sim motor uses a target (training delays 0-0.02 s)'),
        DeclareLaunchArgument(
            'sim_viewer', default_value='false', choices=['true', 'false'],
            description='Open the MuJoCo viewer (needs a display)'),
        DeclareLaunchArgument(
            'sim_stream_to', default_value='',
            description='host:port to stream sim state to tools/walker/sim_viewer.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('gamepad_node'), 'launch', 'gamepad_node.launch.py'])),
            launch_arguments={'input_mode': LaunchConfiguration('input_mode')}.items()),
        OpaqueFunction(function=launch_pipeline),
    ])
