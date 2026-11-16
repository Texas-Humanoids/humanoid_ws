"""Everything the Foxglove dashboard needs, on top of a running robot (or sim):

  foxglove_bridge        WebSocket server Foxglove connects to (ws://<robot>:8765)
  robot_state_publisher  URDF on /robot_description, joint TFs from /joint_states
  robot_monitor          battery, power, tilt, /diagnostics, world -> base_link
  mjpeg_camera           camera_url (MJPEG over HTTP) -> /camera/image/compressed

robot.launch.py includes this with foxglove:=true (the default). On its own:
  ros2 launch th_foxglove foxglove.launch.py \
    robot_config:=src/th_bringup/config/walker_v2_robot.yaml
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def package_uris(urdf, path):
    """Points `package://<robot>/...` meshes at the installed th_description.

    The training URDF names its meshes package://walker_v2/meshes/..., but walker_v2 is a
    folder inside th_description, not a ROS package. foxglove_bridge and RViz resolve
    package:// through the ROS package index, so rewrite to package://th_description/robots/...
    """
    robot_dir = os.path.dirname(os.path.dirname(os.path.abspath(path)))
    robot = os.path.basename(robot_dir)
    return urdf.replace(f'package://{robot}/', f'package://th_description/robots/{robot}/')


def motor_namespace(joint):
    return '/' + joint.removesuffix('_joint')


def launch_dashboard(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    actions = []
    robot = {}
    if arg('robot_config'):
        with open(arg('robot_config')) as f:
            robot = yaml.safe_load(f) or {}
    motors = robot.get('motors') or []
    imu = robot.get('imu') or {}
    battery = robot.get('battery') or {}

    urdf_path = arg('urdf') or os.path.join(
        get_package_share_directory('th_description'),
        'robots', 'walker_v2', 'urdf', 'walker_v2.urdf')
    with open(urdf_path) as f:
        urdf = package_uris(f.read(), urdf_path)
    # The CAD export's dark grey (0.25) nearly disappears on Foxglove's dark background.
    urdf = urdf.replace('rgba="0.25 0.25 0.27 1"', 'rgba="0.62 0.64 0.68 1"')
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}], output='log'))

    monitor = {
        'battery_cells': int(battery.get('cells') or 0),
        'battery_low_cell_voltage': float(battery.get('low_cell_voltage', 3.3)),
        'imu_rotation': [float(v) for v in imu.get('rotation', [1.0, 0.0, 0.0, 0.0])],
        'imu_world_frame': imu.get('world_frame', 'enu'),
        'imu_topic': arg('imu_topic'),
    }
    if motors:  # ROS cannot type an empty list
        monitor['joints'] = [m['joint'] for m in motors]
        monitor['motor_namespaces'] = [motor_namespace(m['joint']) for m in motors]
    actions.append(Node(
        package='th_foxglove', executable='robot_monitor', parameters=[monitor],
        output='screen'))

    if arg('camera_url'):
        actions.append(Node(
            package='th_foxglove', executable='mjpeg_camera',
            parameters=[{'url': arg('camera_url'), 'max_rate': float(arg('camera_rate'))}],
            output='screen'))

    try:
        get_package_share_directory('foxglove_bridge')
    except PackageNotFoundError:
        actions.append(LogInfo(msg=(
            'foxglove_bridge is not installed, so Foxglove cannot connect. Rebuild the image '
            '(docker compose --profile robot build) or apt install ros-jazzy-foxglove-bridge.')))
        return actions
    actions.append(Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{
            'port': int(arg('port')),
            'address': '0.0.0.0',
            # Joint states, TF and the camera only need the latest message; a slow Wi-Fi
            # link then drops old ones instead of falling further behind.
            'max_qos_depth': 10,
        }],
        output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config', default_value='',
            description='Robot yaml (motors, imu, battery), e.g. th_bringup walker_v2_robot.yaml'),
        DeclareLaunchArgument(
            'urdf', default_value='',
            description="URDF for the 3D view; default th_description's walker_v2"),
        DeclareLaunchArgument('port', default_value='8765', description='foxglove_bridge port'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument(
            'camera_url', default_value='',
            description="MJPEG stream to show, e.g. http://localhost:8080/stream; '' = none"),
        DeclareLaunchArgument(
            'camera_rate', default_value='10.0', description='Max camera frames per second'),
        OpaqueFunction(function=launch_dashboard),
    ])
